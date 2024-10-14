THRESHOLD = (0,100)
BINARY_VISIBLE = False

import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot().binary(THRESHOLD) if BINARY_VISIBLE else sensor.snapshot()

    line = img.get_regression([(255,255) if BINARY_VISIBLE else THRESHOLD], robust = True)

    if line:
        img.draw_line(line.line(), color = 127)  # Fixed typo
    print("FPS %f, mag = %s" % (clock.fps(), str(line.magnitude()) if line else "N/A"))  # Fixed condition
