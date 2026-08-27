import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence

import serial


class NMEASerialReader(Node):
    """
    Reads NMEA sentences off a serial port and republishes them verbatim on nmea_sentence.

    This replaces nmea_navsat_driver's nmea_topic_serial_reader, which is broken: it assigns
    the raw bytes from pyserial straight into Sentence.sentence. That field is a rosidl string,
    so the conversion layer aborts the process on the first sentence with
    "Assertion `PyUnicode_Check(field)' failed" rather than raising something catchable. Its
    sibling nmea_serial_driver decodes correctly, but keeps the sentences to itself.

    No NMEA parsing happens here. nmea_navsat_driver's nmea_topic_driver subscribes to what we
    publish and does all of it, including mapping GGA fix quality onto the NavSatFix covariance
    that the EKFs consume. Publishing the raw sentences lets other nodes read satellite count,
    HDOP and fix quality directly instead of inferring them from that covariance.
    """

    def __init__(self):
        super().__init__("nmea_serial_reader")

        # Defaults match the receiver on the robot, so that running this bare with
        # "ros2 run utils nmea_serial_reader" works without arguments.
        self.port = self.declare_parameter("port", "/dev/ttyACM0").value
        self.baud = self.declare_parameter("baud", 9600).value
        self.frame_id = self.declare_parameter("frame_id", "gps_link").value

        self.sentence_pub = self.create_publisher(Sentence, "nmea_sentence", 10)

        self.serial_port = None
        self.serial_port = serial.Serial(port=self.port, baudrate=self.baud, timeout=2)
        self.get_logger().info(f"Reading NMEA sentences from {self.port} at {self.baud} baud.")

    def read_forever(self):
        while rclpy.ok():
            line = self.serial_port.readline().strip()

            if not line:
                continue    # read timed out, nothing on the wire this cycle

            try:
                sentence = line.decode("utf-8")
            except UnicodeDecodeError:
                # Binary noise, which usually means the wrong baud rate or a receiver that
                # has been put into a binary mode. Drop the line instead of the node.
                self.get_logger().warn(
                    "Discarding undecodable bytes from the GPS. Check that the baud rate matches.",
                    throttle_duration_sec=5.0)
                continue

            msg = Sentence()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.sentence = sentence
            self.sentence_pub.publish(msg)

            # The read above blocks, so nothing else gets a turn unless we give it one.
            # Without this, parameter and service calls against this node never answer.
            rclpy.spin_once(self, timeout_sec=0)

    def destroy_node(self):
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = NMEASerialReader()
        node.read_forever()
    except KeyboardInterrupt:
        pass
    except serial.SerialException as e:
        rclpy.logging.get_logger("nmea_serial_reader").fatal(f"Could not read the GPS serial port: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
