# This file is part of the OpenMV project.
# Copyright (c) 2013-2017 Ibrahim Abdelkader <iabdalkader@openmv.io> & Kwabena W. Agyeman <kwagyeman@openmv.io>
# Support for OpenMV Motor Shield by Chris Anderson, DIY Robocars
# This work is licensed under the MIT license, see the file LICENSE for details.

import sensor, image, time, math

###########
# Settings
###########

COLOR_LINE_FOLLOWING = False # False to use grayscale thresholds, true to use color thresholds.

COLOR_THRESHOLDS = [( 94, 100,  -27,    1,   20,  127)] # Yellow Line.
GRAYSCALE_THRESHOLDS = [(240, 255)] # White Line.
COLOR_HIGH_LIGHT_THRESHOLDS = [(80, 100, -10, 10, -10, 10)]
GRAYSCALE_HIGH_LIGHT_THRESHOLDS = [(250, 255)]
BINARY_VIEW = False  # Helps debugging but costs FPS if on.
DO_NOTHING = False # Just capture frames...
FRAME_SIZE = sensor.QQVGA # Frame size.
FRAME_REGION = 0.75 # Percentage of the image from the bottom (0 - 1.0).
FRAME_WIDE = 1.0 # Percentage of the frame width.

AREA_THRESHOLD = 0 # Raise to filter out false detections.
PIXELS_THRESHOLD = 40 # Raise to filter out false detections.
MAG_THRESHOLD = 4 # Raise to filter out false detections.
MIXING_RATE = 0.9 # Percentage of a new line detection to mix into current steering.

###########
# Setup
###########

FRAME_REGION = max(min(FRAME_REGION, 1.0), 0.0)
FRAME_WIDE = max(min(FRAME_WIDE, 1.0), 0.0)
MIXING_RATE = max(min(MIXING_RATE, 1.0), 0.0)

sensor.reset()
sensor.set_pixformat(sensor.RGB565 if COLOR_LINE_FOLLOWING else sensor.GRAYSCALE)
sensor.set_framesize(FRAME_SIZE)
sensor.set_vflip(False)
sensor.set_hmirror(True)
sensor.set_windowing((int((sensor.width() / 2) - ((sensor.width() / 2) * FRAME_WIDE)), int(sensor.height() * (1.0 - FRAME_REGION)), \
                     int((sensor.width() / 2) + ((sensor.width() / 2) * FRAME_WIDE)), int(sensor.height() * FRAME_REGION)))
sensor.skip_frames(time = 200)
if COLOR_LINE_FOLLOWING: sensor.set_auto_gain(False)
if COLOR_LINE_FOLLOWING: sensor.set_auto_whitebal(False)
clock = time.clock()

###########
# Loop
###########

while True:
    clock.tick()
    img = sensor.snapshot()
    img.binary(COLOR_HIGH_LIGHT_THRESHOLDS if COLOR_LINE_FOLLOWING else GRAYSCALE_HIGH_LIGHT_THRESHOLDS, zero=True)
    img.histeq()

    if BINARY_VIEW: img = img.binary(COLOR_THRESHOLDS if COLOR_LINE_FOLLOWING else GRAYSCALE_THRESHOLDS)
    if BINARY_VIEW: img.erode(1, threshold=5).dilate(1, threshold=1)
    if DO_NOTHING: continue

    # We call get regression below to get a robust linear regression of the field of view.
    # This returns a line object which we can use to steer the robocar.
    line = img.get_regression(([(50, 100, -128, 127, -128, 127)] if BINARY_VIEW else COLOR_THRESHOLDS) if COLOR_LINE_FOLLOWING \
        else ([(127, 255)] if BINARY_VIEW else GRAYSCALE_THRESHOLDS), \
        area_threshold=AREA_THRESHOLD, pixels_threshold=PIXELS_THRESHOLD, \
        robust=True)

    if line and (line.magnitude() >= MAG_THRESHOLD):
        img.draw_line(line.line(), color=(127, 127, 127) if COLOR_LINE_FOLLOWING else 127)
        print("Line Ok - line t: %d, r: %d" % (line.theta(), line.rho()))
    else:
        print("Line Lost")
    print("FPS %f" % clock.fps())
