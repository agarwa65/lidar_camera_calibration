#!/usr/bin/env python

import math
import rospy
import sys
import cv2
import cv_bridge
from image_geometry import PinholeCameraModel
import tf
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import struct
import numpy as np


camera ={}
imageOverlay = {}
cameraInfo = CameraInfo()
cameraModel = PinholeCameraModel()
bridge = cv_bridge.CvBridge()
tf_ = tf.TransformListener()

velodyneData = []


def camera_callback(data):
	global cameraModel, camera

	cameraInfo = data
	print('Receiving Camera Info.. ')

	# Unregister the camera after cameraInfo is received
	# camera.unregister()

	# Set the camera parameters from the sensor_msgs.msg.CameraInfo message
	cameraModel.fromCameraInfo( cameraInfo )

def velodyne_callback(data):
	global velodyneData
	print('In velodyne callback - received point cloud')

	# print("data type :", type( data.data ))
	# print( "data len :",len( data.data ))

	formatString = 'ffff'
	if data.is_bigendian:
		formatString = '>' + formatString
	else:
		formatString = '<' + formatString

	points = []

	for index in range( 0, len( data.data ), 16 ):
		points.append( struct.unpack( formatString, data.data[ index:index + 16 ] ) )

	# print(len( points ))
	velodyneData = points


def image_callback(data):
	global velodyneData, bridge, tf_
	print('In input image callback - received rectified image')

	cv_image = {}

	try:
		cv_image = bridge.imgmsg_to_cv2(data, 'rgb8')
	except cv_bridge.CvBridgeError as e:
			print('Failed to convert image', e)
			return

	# the transform is same for every frame 
	(trans, rot) = tf_.lookupTransform( 'world', 'velodyne', rospy.Time( 0 ) )

	# print("transformation: ", trans, rot)
	trans = tuple(trans) + ( 1,  )
	# print(trans)
	rotationMatrix = tf.transformations.quaternion_matrix( rot )
	# append translation to the last column of rotation matrix(4x4)
	rotationMatrix[ :, 3 ] = trans
	# print('rotationMatrix::  ', rotationMatrix)

	if velodyneData:

		for i in range(0, len(velodyneData) - 1):
			try:
				# converting to homogeneous coordinates
				point = [velodyneData[i][0], velodyneData[i][1], velodyneData[i][2],1]
				# print( point )

				# calculating dist, if dist 4m away, ignore the point
				if math.sqrt( np.sum( np.array( point[ :3 ] ) ** 2 ) ) > 4.0:
					continue

			except IndexError:
				print("Index Error!!!!!")
				break

			#  project 3D point to 2D uv 
			rotatedPoint = rotationMatrix.dot( point )
			uv = cameraModel.project3dToPixel( rotatedPoint )

			# check if the uv point is valid
			if uv[0] >= 0 and uv[0] <= data.width and uv[1] >= 0 and uv[1] <= data.height:
				# Writing on image
				cv2.line(cv_image,(int( uv[0] ),int( uv[1] )),(int( uv[0] )+2,int( uv[1] ) +2),(255,0,0),3)

	try:
		imageOverlay.publish(bridge.cv2_to_imgmsg( cv_image, 'bgr8' ) )
	except cv_bridge.CvBridgeError as e:
		print( 'Failed to convert image', e )
		return


if __name__ == '__main__':
	try:

		# Initialize the node and name it.
		rospy.init_node('lidar_overlay_image')

		# look up camera name, after remapping 
		cameraName = rospy.resolve_name( 'camera' )
		print('Waiting for camera_info from ' + cameraName)

		# look up image_rect_color 
		imageRectName = rospy.resolve_name( 'image' )
		print('Waiting for input image from ' + imageRectName)

		# look up velodyne data
		velodynePointName = rospy.resolve_name('velodyne')
		print('Waiting for velodyne point data from ' + velodynePointName)

		# Subscribe to topic, cameraInfo and callback function.
		camera = rospy.Subscriber( cameraName, CameraInfo, callback = camera_callback)
		imageRect = rospy.Subscriber(imageRectName, Image, callback = image_callback)
		velodynePoint = rospy.Subscriber(velodynePointName, PointCloud2, callback = velodyne_callback)

		# look up lidar overlay image
		imageOverlayName = rospy.resolve_name( 'image_overlay')

		# Publish the lidar overlay image
		imageOverlay = rospy.Publisher( imageOverlayName, Image, queue_size = 1)

		rospy.spin()

	except rospy.ROSInterruptException: pass