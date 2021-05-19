export REALSENSE_ROS_WS=$HOME/projects/ros/ws00
sudo apt install ros-noetic-ddynamic-reconfigure
git clone https://github.com/IntelRealSense/realsense-ros.git $REALSENSE_ROS_WS/src/realsense-ros
cd $REALSENSE_ROS_WS
catkin_make
