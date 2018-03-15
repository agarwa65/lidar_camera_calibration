# Lidar-Camera-Calibration in ROS

1. Install ROS indigo for Ubuntu 14.04

2. Download .bag file 

3. Start roscore 

4. To see the contents of the bag:
```shell
  $ rosbag info 2016-11-22-14-32-13_test.bag 
```

5. Understanding ROS message types in the bag:
```shell
  rosmsg show sensor_msgs/PointCloud2
  rosmsg show sensor_msgs/Image
  rosmsg show sensor_msgs/CameraInfo
```

6. Script 'read_bag.py' to see the values in the rosbag

7. To publish the topics in the bag, run in a new terminal:
```shell
  $ rosbag play 2016-11-22-14-32-13_test.bag
```

  To see published topics:
```shell
  $ rostopic list -v
```

8. To export images from rosbag file, ran the export.launch script
```shell
  $ roslaunch export.launch
```
  It stores images in .ros folder in home directory

9. To see the input video stream from the bag file:
```shell
  $ rosrun image_view image_view image:=/sensors/camera/image_color
```

Now, we know the contents of the ros-bag.


**Reference: ** 
http://wiki.ros.org/ROS/Tutorials
http://wiki.ros.org/rosbag/Cookbook
http://wiki.ros.org/roslaunch/XML/node


# TASK 1: Camera Calibration for Image Rectification


1. Rosbag file is played in one terminal 

2. Publishing Camera Calibration data with bag file using camera_calibration package:
```shell
  $ rosrun camera_calibration cameracalibrator.py --size=7x5 --square=0.050 image:=/sensors/camera/image_color camera:=/sensors/camera/camera_info  --no-service-check
```
Note: --no-service-check added to not wait for '/sensors/camera/camera_info/set_camera_info' 
 

OUTPUT: Stored in ost.yaml file, udpated camera calibration matrices

```yaml
image_width: 964
image_height: 724
camera_name: narrow_stereo
camera_matrix K:
  rows: 3
  cols: 3
  data: [481.228482, 0.000000, 456.782531, 0.000000, 481.158298, 364.412635, 0.000000, 0.000000, 1.000000]
distortion_model : plumb_bob
distortion_coefficients D:
  rows: 1
  cols: 5
  data: [-0.195875, 0.065588, 0.003400, 0.000218, 0.000000]
rectification_matrix R:
  rows: 3
  cols: 3
  data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
projection_matrix P:
  rows: 3
  cols: 4
  data: [422.024750, 0.000000, 460.332694, 0.000000, 0.000000, 429.271149, 368.531509, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
```

3. Saved Images in the calibrationdata folder
Reference: http://wiki.ros.org/camera_calibration

Note: Camera Calibration matrices can also be calculated using opencv apis, once the images are collected from bag file using export.launch script.

4. For image rectification, camera calibration matrices info needs to be added to the input bag file. Using ros bag_tools 'change_camera_info.py', create a new output ros bag with updated calibration matrices.

```shell
$ rosrun bag_tools change_camera_info.py ../2016-11-22-14-32-13_test_orig.bag ../2016-11-22-14-32-13_test_cameracalibrated_out.bag /sensors/camera/camera_info=../calibrationdata/ost.yaml
```

5. Checked using 'read_bag.py' script that the values are updated.

6. To rectify images, image_proc is used. The image_proc_view.launch file is used to read the calibrated bag file and rectify the stream

# Explanation:

image_proc subscribes to image_color and camera_info and publishes image_rect_color

```xml
  <node name="cam_image_proc" pkg="image_proc" type="image_proc" respawn="false" ns="/sensors/camera">
  </node>
```

To check if image proc is publishing image_rect_color:

```shell
$ rostopic info /sensors/camera/image_rect_color
Type: sensor_msgs/Image

Publishers: 
 * /sensors/camera/cam_image_proc 

Subscribers: 
 * /extract_rect_image 
```
image_view subscribes to image_rect_color and extracts the rectified images from the stream
```xml
  <node name="extract_rect_image" pkg="image_view" type="extract_images" respawn="false" required="true" output="screen" cwd="ROS_HOME">
       <remap from="image" to="/sensors/camera/image_rect_color"/>
  </node>
```
image_view stores file in home directory in .ros folder, few randomly chosen rectified images were stored in output directory, as it creates a lot of images for the whole bag file



