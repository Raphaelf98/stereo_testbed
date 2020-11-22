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





