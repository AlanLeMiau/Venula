from picamera import PiCamera
from time import sleep

cam = PiCamera()

cam.start_preview()
sleep(5)
cam.stop_preview()

