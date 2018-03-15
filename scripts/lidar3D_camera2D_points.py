#!/usr/bin/env python

import math
import random
import commentjson as json
import time
import tf
import rospy
import sys
import cv2
import cv_bridge
import numpy as np
from scipy.optimize import minimize
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo

params = {}
cameraInfo = CameraInfo()
cameraModel = PinholeCameraModel()
camera = {}

def cost_function(x0):
	global cameraModel, params

	# state_vars = [tx ty tz yaw pitch roll]
	state_vars = x0
	# print('')
	# print(state_vars)
	translation = [ state_vars[ 0 ], state_vars[ 1 ], state_vars[ 2 ], 1.0 ]

	# euler_matrix format roll, pitch, yaw angles
	rotationMatrix = tf.transformations.euler_matrix( state_vars[ 5 ], state_vars[ 4 ], state_vars[ 3 ] )
	rotationMatrix[ :, 3 ] = translation


	error = 0
	for i in range( 0, len( params[ 'points' ] ) ):

		point = params['points'][ i ]
		input_uv = params['uvs'][ i ]

		# rotate the input point and project it to get uv space
		rotatedPoint = rotationMatrix.dot( point )
		uv = cameraModel.project3dToPixel( rotatedPoint )


		# calculate error between expected uv and calculated uv
		diff = np.array( uv ) - np.array( input_uv )
		error = error + math.sqrt( np.sum( diff * diff ) )
	print('')
	print(uv)
	print(diff)
	print(error)

	return error

def camera_callback(data):
	global cameraModel, camera, params

	cameraInfo = data
	print('Receiving Camera Info.. ')

	# Unregister the camera after cameraInfo is received
	camera.unregister()

	# Set the camera parameters from the sensor_msgs.msg.CameraInfo message
	cameraModel.fromCameraInfo( cameraInfo )

	print('Start optimization..')
	result = minimize( cost_function, params[ 'initTransform' ], args = ( ), bounds = params[ 'bounds' ], method = 'SLSQP', options = { 'disp': True, 'maxiter': 500 } )

	# run till the optimization returns no success or if the obj function value >45
	while not result.success or result.fun > 45:
		for i in range( 0, len( params[ 'initTransform' ] ) ):
			# choose random state vector from within the bounds
			params[ 'initTransform' ][ i ] = random.uniform( params[ 'bounds' ][i][0], params[ 'bounds' ][i][1] )

		print( '' )
		print( 'Trying new starting point:' )
		print( params[ 'initTransform' ] )
		result = minimize( cost_function, params[ 'initTransform' ], args = ( ), bounds = params[ 'bounds' ], method = 'SLSQP', options = { 'disp': True, 'maxiter': 500 } )

	print('Finished 2D-3D correspondences calculation..' )
	print('Final static transform :' )
	print( result.x )
	# value of the objective function
	print('Error: ' + str( result.fun ))

	sys.stdout.flush()
	rospy.signal_shutdown( 'Finished 2D-3D correspondences calculation' )


if len(sys.argv) > 1:
	paramsFile = open(sys.argv[ 1 ], 'r')

# load json 3D-2D points and other parameters
params = json.load(paramsFile)
print(params)


if __name__ == '__main__':
	try:
		# Initialize the node and name it.
		rospy.init_node('lidar3D_camera2D_points')

		# look up camera name, after remapping 
		cameraName = rospy.resolve_name( 'camera' )
		print('Waiting for camera_info from ' + cameraName)

		# Subscribe topic, cameraInfo and callback function.
		camera = rospy.Subscriber( cameraName, CameraInfo, callback = camera_callback)
		rospy.spin()

	except rospy.ROSInterruptException: pass
