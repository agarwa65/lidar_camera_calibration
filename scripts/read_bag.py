# USAGE:  python read_bag.py ../2016-11-22-14-32-13_test.bag
import rosbag,sys

bag = rosbag.Bag(sys.argv[1])

for topic,msg,t in bag.read_messages(topics=['/sensors/camera/camera_info']):
	print msg
	# print msg.D 
	# print msg.K
bag.close()



             
