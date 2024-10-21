import sensor, image, time, ustruct
from pyb import USB_VCP

usb = USB_VCP()
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.

while(True):
    cmd = usb.recv(4, timeout=5000)
    if (cmd == b'snap'):
        img = sensor.snapshot().compress()
        usb.send(ustruct.pack("<L", img.size()))
        usb.send(img)
