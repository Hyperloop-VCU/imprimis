#!/usr/bin/env python3

'''
This python script acts as an NTRIP client, connecting to an NTRIP caster and receiving RTCM data.

The client will first connect to the GPS via serial port, then connect to the NTRIP Caster.
It will forward GGA messages from the GPS to the caster, recieve RTCM datA, and send that back to the GPS via serial port.

Additionally, the client will publish the GPS' GGA messages to a ROS2 topic called 'nmea_sentences' which is subscribed to
by the nmea navsatdriver to convert those into NavSatFix messages which get published to /gps/fix

The NTRIP Configuration section allows you to specify the connection details.
Note that to login for some mountpoints you may need a username and password.

While the details are defined below, it is also possible to override them using CLARGS.
'''

from ntrip_client.login_params import USERNAME_AUTH, PASSWORD_AUTH  #This stores credentials. Github doesn't like it when I upload 'secrets'

# ==============================
# NTRIP AUTHENTICATION CONFIGURATION
# ==============================
                                #Default is:
CASTER = "rtk.geodnet.com"      #rtk.geodnet.com
PORT = 2101                     #2101
MOUNTPOINT = "AUTO"             #AUTO

USERNAME = USERNAME_AUTH
PASSWORD = PASSWORD_AUTH

OUTPUT_FILE = ""
# ==============================
# DEBUG CONFIG
DEBUG = False

# ==============================
# GPS/NTRIP CONFIGURATION
RATE = 1.0  # Exchanges GPS data and correction data at this interval.
SOCKET_TIMEOUT = 1
# ==============================

#=================================
#GPS PORT CONFIG
GPS_PORT = "/dev/ttyACM0" #Check this
BAUD_RATE = 38400
#==================================

import socket
import base64
import argparse
import select

import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence
from std_msgs.msg import String
import serial

import ntrip_client.map_UI as GPS

#Define class header
class NTRIPClient(Node):

    def __init__(
        self
    ):
        
        super().__init__('ntrip_client')

                # ==============================
        # ROS PARAMETERS
        # ==============================

        self.declare_parameter('caster', CASTER)
        self.declare_parameter('port', PORT)
        self.declare_parameter('mountpoint', MOUNTPOINT)
        self.declare_parameter('username', USERNAME)
        self.declare_parameter('password', PASSWORD)

        self.declare_parameter('gps_port', GPS_PORT)
        self.declare_parameter('baud_rate', BAUD_RATE)

        self.declare_parameter('rate', RATE)
        self.declare_parameter('socket_timeout', SOCKET_TIMEOUT)

        self.declare_parameter('debug', DEBUG)
        self.declare_parameter('show_map', False)
        self.declare_parameter('sim', False)


        # Get parameters
        self.caster = self.get_parameter('caster').value
        self.port = self.get_parameter('port').value
        self.mountpoint = self.get_parameter('mountpoint').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value

        self.gps_port = self.get_parameter('gps_port').value
        self.baud_rate = self.get_parameter('baud_rate').value

        self.rate = self.get_parameter('rate').value
        self.socket_timeout = self.get_parameter('socket_timeout').value

        self.debug = self.get_parameter('debug').value
        self.show_map = self.get_parameter('show_map').value

        self.sim = self.get_parameter('sim').value



        self.sock = None
        self.file = None
        self.serial_port = None
        self.latest_gga = None


        #Create publisher for NMEA sentences for navsatdriver
        self.gga_publisher = self.create_publisher(
            Sentence,
            'nmea_sentences',
            10,
        )

        if self.sim:
            self.gga_subscription = self.create_subscription(
                String,
                '/gps/gga',
                self._simulated_gga_callback,
                10,
            )

        # Define port to connect to GPS device if available, otherwise SIM

        if not self.sim:
            try:
                self.serial_port = serial.Serial(
                    port=self.gps_port,
                    baudrate=self.baud_rate,
                    timeout=0.1,
                )
                self.get_logger().info(f"Connected to GPS serial port: {self.gps_port} at {self.baud_rate} baud.")
            except Exception as exc:
                self.get_logger().warning(
                    f"GPS serial port {self.gps_port} unavailable: {exc}"
                )
        else:
            self.get_logger().info("Using simulated GGA messages from /gps/gga.")

        
        self.gps_plotter = None

        #If map enabled, will create and start a GPS plotter
        if self.show_map:
            self.gps_plotter = GPS.GPSPlotter()
            self.gps_plotter.start()

        # Create timer to receive RTCM corrections at the specified rate.
        self.gga_timer = self.create_timer(RATE, self.run)

    #Connects to NTRIP Caster and authenticates with provided credentials.
    def connect(self):
        if self.sock is not None:
            return

        self.get_logger().info(f"Connecting to {self.caster}:{self.port}...")

        #Create socket
        self.sock = socket.create_connection(
            (self.caster, self.port),
            timeout=SOCKET_TIMEOUT
        )

        # Build authentication header
        headers = [
            f"GET /{self.mountpoint} HTTP/1.0",
            "User-Agent: NTRIP PythonClient/1.0",
            "Accept: */*",
            "Connection: close",
        ]

        #Compile authenication header
        if self.username is not None:
            credentials = f"{self.username}:{self.password or ''}"
            encoded = base64.b64encode(
                credentials.encode()
            ).decode()

            headers.append(
                f"Authorization: Basic {encoded}"
            )

        request = "\r\n".join(headers) + "\r\n\r\n"

        if DEBUG:
            self.get_logger().info("Sending NTRIP HTTP request.")
            self.get_logger().info(request)

        #Send Request to Caster
        self.sock.sendall(request.encode())

        # Read caster response
        response = b""
        try:
            while b"\r\n\r\n" not in response:
                data = self.sock.recv(4096)

                if not data:
                    break

                response += data

                if len(response) > 16384:
                    break
        except socket.timeout as exc:
            self.close()
            raise TimeoutError(f"Timed out waiting for NTRIP caster response: {exc}") from exc

        header, separator, remaining = response.partition(b"\r\n\r\n")

        
        self.get_logger().info("Caster response:")
        self.get_logger().info("--------------------------------")
        self.get_logger().info(header.decode(errors="replace"))
        self.get_logger().info("--------------------------------")

        # Check response
        first_line = header.split(b"\r\n")[0].decode(
            errors="replace"
        )

        #Error Messages if Caster didn't connect/ rejected authentication
        if (
            "200 OK" not in first_line
            and "ICY 200 OK" not in first_line
        ):
            self.close()
            raise RuntimeError(
                f"NTRIP caster rejected connection: {first_line}"
            )

        self.get_logger().info("NTRIP connection established.")

        # The bytes after the HTTP header may already contain
        # the beginning of the RTCM stream.
        if remaining:
            self.process_data(remaining)


    # Send RTCM corrections to the GPS serial port if available,
    # otherwise save them to file if configured.
    def process_data(self, data):

        #Check if port available, then send
        if self.serial_port is not None:
            try:
                self.serial_port.write(data)
                self.serial_port.flush()
                if DEBUG:
                    self.get_logger().info(f"[IMPORTANT] Sent RTCM bytes to GPS serial port: {len(data)}")
                return
            except Exception as exc:
                self.get_logger().warning(f"Failed to write RTCM data to GPS serial port: {exc}\n Bruh.")

        if self.file:
            self.file.write(data)
            self.file.flush()

    def _update_latest_gga(self):
        """
        Will read serial port and obtain GGA messages and set that as the latest GGA message
        """

        if self.serial_port is None:
            return

        try:
            while True:

                #Read serial input
                line = self.serial_port.readline()
                if not line:
                    break

                #Indeitifies message type as GGA.
                sentence = line.decode("ascii", errors="ignore").strip()
                self._process_gga_sentence(sentence)
    

                if not self.serial_port.in_waiting:
                    break
        except Exception as exc:
            self.get_logger().error(f"Failed to read GPS serial port: {exc}")

    def _simulated_gga_callback(self, msg):
        self._process_gga_sentence(msg.data)

    def _process_gga_sentence(self, sentence):
        sentence = sentence.strip()
        sentence_id = (
            sentence[1:].partition(",")[0]
            if sentence.startswith("$")
            else ""
        )
        if not sentence_id.endswith("GGA"):
            if DEBUG:
                self.get_logger().warning(
                    f"Ignoring non-GGA sentence: {sentence_id}"
                )
            return

        self.latest_gga = f"{sentence}\r\n"

        gga_msg = Sentence()
        gga_msg.header.stamp = self.get_clock().now().to_msg()
        gga_msg.header.frame_id = 'gps'
        gga_msg.sentence = sentence
        self.gga_publisher.publish(gga_msg)

        #Send info to plotter if enabled.
        if self.show_map and self.gps_plotter is not None:
            self.gps_plotter.update_gga(sentence)
    

    def run(self):
        """Forward GPS serial data to the caster and RTCM back to the GPS."""

        self._update_latest_gga()

        #Connect if not currecntly connected to client
        if self.sock is None:
            self.get_logger().info("No socket yet; trying to connect to NTRIP caster.")
            try:
                self.connect()
            except Exception as exc:
                self.get_logger().error(f"Unable to connect to NTRIP caster: {exc}")
                return

        try:
            if self.latest_gga is not None:

                #Run ext functions for showing GPS location
                #

                gga_data = self.latest_gga.encode("ascii")
                self.sock.sendall(gga_data)
                if DEBUG:
                    self.get_logger().info(
                        f"Sent latest GGA to NTRIP caster: {len(gga_data)} bytes"
                    )


                #Read returned RTCM data. If valid, forward to GPS.
                try:
                    data = self.sock.recv(4096)
                except (BlockingIOError, socket.timeout):
                    return

                if not data:
                    self.get_logger().warn("Caster closed the connection.")
                    self.close()
                    return

                self.get_logger().info(f"Received RTCM bytes: {len(data)}")
                self.process_data(data)

        except KeyboardInterrupt:
            self.get_logger().info("Stopping...")
            self.close()
        except Exception as exc:
            self.get_logger().error(f"NTRIP run loop failed: {exc}")
            self.close()
    
    #Shut down NTRIP Client
    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass

            self.sock = None

        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None

        if self.file:
            self.file.close()
            self.file = None

        self.get_logger().info("Connection closed.")

def main(args=None):
    rclpy.init(args=args)

    client = NTRIPClient()


    rclpy.spin(client)

    client.close()
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()