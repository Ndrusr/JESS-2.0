# JESS-2.0
Jessica 2.0 is a 6-Axis robotic arm project by the SUTD Organization of Autonomous Robotics (SOAR)

# Installation
## Part 0: Ubuntu 20.04

## Part 1: ROS Noetic

## Part 2: MoveIt

## Part 3: Intel RealSense
To install realsense SDK on your computer as well as the ROS wrapper, do the following:
```
chmod +x realsense_install.sh
chmod +x realsense_ros_install.sh
./realsense_install.sh
./realsense_ros_install.sh
```

To uninstall:
```
chmod +x realsense_uninstall.sh
./realsense_uninstall.sh
```

To run RealSense Viewer:
```
/opt/realsense/bin/realsense-viewer
```

To visualize RealSense data on RVIZ:  
In a terminal window:
```
cd projects/ros/ws00/
roslaunch realsense2_camera rs_camera.launch
```

In another terminal window:
```
rosrun rviz rviz
```

In RVIZ, under Displays, Global Options, change the Fixed Frame parameter, using the drop down menu, to 'camera_link'

Click add & under /camera, /depth, /image_rect_raw, double click the DepthCloud option with the 'raw' parameter set in the drop down menu. And at long last, the depth cloud from the RealSense camera should be displayed.
