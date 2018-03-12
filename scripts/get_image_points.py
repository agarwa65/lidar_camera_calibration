#! /usr/bin/env python

import cv2
import rospy

def mouse_callback(event, x, y, flags, params):
	if event == cv2.EVENT_LBUTTONUP:
		print(x, y)

filename = "/home/divya/catkin_ws/src/auro_assignment/images/frame0000.jpg"
image = cv2.imread(filename)
print(image.shape)
cv2.namedWindow("image")
cv2.setMouseCallback("image", mouse_callback)
cv2.imshow("image", image)
q = cv2.waitKey(0)
cv2.destroyAllWindows()

