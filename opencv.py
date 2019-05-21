#!/usr/bin/env python
import cv2
import picamera
import numpy as np
import time

with picamera.PiCamera() as camera:
	camera.resolution = (320, 240)
	camera.framerate = 24
	time.sleep(2)

	while True:
		image = np.empty((240 * 320 *3,), dtype=np.uint8)
		camera.capture(image, format='bgr')
		image = image.reshape((240, 320, 3))

		cv2.imshow("img", image)
		cv2.waitKey(1)
