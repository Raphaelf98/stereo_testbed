<!-- Headings -->
# stereo_camera_testbed
This repository contains software which has been developed for a bachelor thesis.

The package has been tested under ROS Noetic on Ubuntu 20.04. This is research code and any fitness for a particular purpose is disclaimed. 

## Prerequisites
This code is embedded within the ROS framework.
For this reason make sure ROS Noetic is installed on Ubuntu 20.04 and your catkin workspace is set up. 

Go to [ROS-Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu) if you need to install ROS Noetic. 

The repository is organized as a ROS package and its functionality depends on topics published other ROS packages. 
For this reason make sure topics provided in [ROS-image_pipeline](http://wiki.ros.org/image_pipeline?distro=noetic) packages are available. 

The [stereo_image_proc](http://wiki.ros.org/stereo_image_proc?distro=noetic) within the image_pipeline package is dependent on camera image- and camera information topics as well as calibrated stereo cameras. 

Stereo calibration can be done with the [camera_calibration](http://wiki.ros.org/camera_calibration?distro=noetic) package provided in image_pipeline. 
 
## How to use?

Run stereo camera calibration tool.
```
rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 7x6 --square 0.015 right:=/stereo/right/image_raw left:=/stereo/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/left
```
Adjust size and square length according to the used chessboard and its dimensions. 

If camera calibration was successful and calibration data is stored, run the stereo_image_proc node with the following command:
```
ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc approximate_sync:=true queue_size= 20
```
Next, run 
```
rviz
```
and subscribe to the points2 topic published by the stereo_image_proc node.
Adjustments to the stereo matching algorithms can be done with the help of rqt configure. For this, run the following command from a new terminal.
```
rosrun rqt_reconfigure rqt_reconfigure
```
### extrinsicCalibration node
Three dimensional backbone points can be processed in a predefined frame by doing an extrinsic calibration with the help of a chessboard pattern.
By running
```
rosrun stereo_testbed extrinsicCalibration
```
the coordinate transformation between camera frame and chessboard frame origin is computed. 

### skeleton node
By starting the skeleton node, extrinisic calibration data is used to express all backbone points in the coordinate frame defined by the user.
```
rosrun stereo_testbed skeleton 
```
Calculated three dimensional backbone points are published under the SkeletonPoints topic.

### parametricCurveFitting node
Inside the parametricCurveFitting node functionality for ordering and approximating backbone points provided by the SkeletonPoints topic.
At the current state the node is adjusted for single execution and therefore it will use the current backbone points published by the skeleton node. 
Points are sorted with the help of Dijkstra's algorithm and approximated by a third degree B-spline. Notice that the source for the Dijstra sort is given by the point with the lowest X-coordinate. 
```
rosrun stereo_testbed parametricCurveFitting.py 
```






