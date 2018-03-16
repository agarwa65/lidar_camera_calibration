#! /usr/bin/env python

import cv2
import rospy

refPt = []
filename = "/home/divya/catkin_ws/src/auro_assignment/images/frame0000.jpg"
image = cv2.imread(filename)

def show_image():
	for i in range(0,len(refPt)):
		cv2.circle(image,refPt[i], 2,(255,0,0),3)
	# cv2.imshow("image1", image)
	cv2.imwrite("alpha.png", image)



def mouse_callback(event, x, y, flags, params):
	if event == cv2.EVENT_LBUTTONUP:
		refPt.append((x,y))
		print(x, y)



# filename = "/home/divya/catkin_ws/src/auro_assignment/images/frame0000.jpg"
# image = cv2.imread(filename)
print(image.shape)
cv2.namedWindow("image")
cv2.setMouseCallback("image", mouse_callback)


while True:
	cv2.imshow("image", image)
	key = cv2.waitKey(1) & 0xFF

	# if the 'c' key is pressed, break from the loop
	if key == ord("c"):
		break

show_image()
q = cv2.waitKey(0)

cv2.destroyAllWindows()

