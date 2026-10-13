#!/usr/bin/env python3

from flask import Flask, jsonify, render_template_string
import threading
import webbrowser
import time


class GPSPlotter:

    def __init__(self, host="0.0.0.0", port=5000):

        self.host = host
        self.port = port

        # Latest GPS position
        self.latitude = None
        self.longitude = None
        self.altitude = None

        self.fix_quality = None
        self.satellites = None
        self.hdop = None

        # Protect data shared between
        # ROS thread and Flask thread
        self.lock = threading.Lock()

        # Flask application
        self.app = Flask(
            __name__
        )

        self._setup_routes()

        self.server_thread = None


    # ========================================================
    # NMEA coordinate conversion
    # ========================================================

    @staticmethod
    def nmea_to_decimal(
        coordinate,
        direction
    ):

        value = float(coordinate)

        degrees = int(
            value // 100
        )

        minutes = (
            value
            - degrees * 100
        )

        decimal = (
            degrees
            + minutes / 60.0
        )

        if direction in ("S", "W"):
            decimal *= -1

        return decimal


    # ========================================================
    # Give a GGA sentence to the plotter
    # ========================================================

    def update_gga(self, gga):

        """
        Update the map using a GGA sentence.

        Example:

        $GNGGA,123519,3742.1234,N,07726.1234,W,4,18,0.8,52.3,M,...
        """

        parts = gga.strip().split(",")

        if len(parts) < 10:
            return

        # Make sure this is GGA
        if not parts[0].endswith("GGA"):
            return

        try:

            latitude = self.nmea_to_decimal(
                parts[2],
                parts[3]
            )

            longitude = self.nmea_to_decimal(
                parts[4],
                parts[5]
            )

            fix_quality = int(
                parts[6]
            )

            satellites = int(
                parts[7]
            )

            hdop = float(
                parts[8]
            )

            altitude = float(
                parts[9]
            )

        except (ValueError, IndexError):

            return


        # ----------------------------------------------------
        # Update shared GPS data
        # ----------------------------------------------------

        with self.lock:

            self.latitude = latitude
            self.longitude = longitude
            self.altitude = altitude

            self.fix_quality = fix_quality
            self.satellites = satellites
            self.hdop = hdop


    # ========================================================
    # Flask routes
    # ========================================================

    def _setup_routes(self):

        @self.app.route("/")
        def index():

            return render_template_string(
                MAP_HTML
            )


        @self.app.route("/position")
        def position():

            with self.lock:

                return jsonify({

                    "latitude":
                        self.latitude,

                    "longitude":
                        self.longitude,

                    "altitude":
                        self.altitude,

                    "fix_quality":
                        self.fix_quality,

                    "satellites":
                        self.satellites,

                    "hdop":
                        self.hdop

                })


    # ========================================================
    # Start live map
    # ========================================================

    def start(self, open_browser=True):

        self.server_thread = threading.Thread(

            target=self._run_server,

            daemon=True

        )

        self.server_thread.start()


        # Give Flask a moment to start

        if open_browser:

            threading.Thread(

                target=self._open_browser,

                daemon=True

            ).start()


    # ========================================================
    # Flask server
    # ========================================================

    def _run_server(self):

        self.app.run(

            host=self.host,

            port=self.port,

            debug=False,

            use_reloader=False

        )


    # ========================================================
    # Open browser
    # ========================================================

    def _open_browser(self):

        time.sleep(1)

        webbrowser.open(
            f"http://localhost:{self.port}"
        )


# ============================================================
# HTML / JavaScript map
# ============================================================

MAP_HTML = """

<!DOCTYPE html>

<html>

<head>

    <meta charset="UTF-8">

    <title>Live GPS Position</title>


    <meta
        name="viewport"
        content="width=device-width,
                 initial-scale=1.0"
    >


    <link
        rel="stylesheet"
        href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
    >


    <style>

        html, body {

            width: 100%;
            height: 100%;

            margin: 0;
            padding: 0;

        }


        #map {

            width: 100%;
            height: 100%;

        }


        #info {

            position: absolute;

            top: 10px;
            left: 60px;

            z-index: 1000;

            background: white;

            padding: 12px;

            border-radius: 6px;

            font-family: Arial;

            box-shadow:
                0 2px 6px
                rgba(0,0,0,0.3);

        }

    </style>

</head>


<body>


<div id="info">

    <b>GPS Position</b>

    <div id="gps">

        Waiting for GPS...

    </div>

</div>


<div id="map"></div>


<script
    src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js">
</script>


<script>


// ============================================================
// Map
// ============================================================

const map = L.map(
    "map"
).setView(

    [37.5407, -77.4360],

    15

);


// ============================================================
// OpenStreetMap
// ============================================================

L.tileLayer(

    "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",

    {

        maxZoom: 19,

        attribution:
            "&copy; OpenStreetMap contributors"

    }

).addTo(map);


// ============================================================
// GPS marker
// ============================================================

let marker = null;


// ============================================================
// Update GPS
// ============================================================

async function updateGPS()
{

    try {

        const response =
            await fetch("/position");


        const data =
            await response.json();


        if (
            data.latitude === null ||
            data.longitude === null
        ) {

            document.getElementById(
                "gps"
            ).innerHTML =
                "Waiting for GPS fix...";

            return;

        }


        const position = [

            data.latitude,

            data.longitude

        ];


        // ----------------------------------------------------
        // Create marker
        // ----------------------------------------------------

        if (marker === null) {

            marker = L.marker(
                position
            ).addTo(map);


            map.setView(
                position,
                18
            );

        }

        else {

            marker.setLatLng(
                position
            );

        }


        // ----------------------------------------------------
        // Keep map centered
        // ----------------------------------------------------

        map.panTo(
            position
        );


        // ----------------------------------------------------
        // Display GPS information
        // ----------------------------------------------------

        document.getElementById(
            "gps"
        ).innerHTML =

            "Latitude: " +
            data.latitude.toFixed(8) +

            "<br>" +

            "Longitude: " +
            data.longitude.toFixed(8) +

            "<br>" +

            "Altitude: " +
            data.altitude.toFixed(2) +
            " m" +

            "<br>" +

            "Satellites: " +
            data.satellites +

            "<br>" +

            "HDOP: " +
            data.hdop +

            "<br>" +

            "Fix: " +
            data.fix_quality;

    }

    catch (error) {

        console.error(
            error
        );

    }

}


// ============================================================
// Update twice per second
// ============================================================

setInterval(
    updateGPS,
    500
);


// Initial update

updateGPS();


</script>


</body>

</html>

"""