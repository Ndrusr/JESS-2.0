# How to Calibrate your Intel RealSense for MoveIt

Navigate to the src folder of your ROS workspace.
```
git clone https://github.com/ros-planning/moveit_calibration.git
```

```
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
catkin build
source devel/setup.sh
```

```
roslaunch panda_moveit_config demo.launch
```

Panels, add new panel, HandEyeCalibration, ok.

Create and print target.

Set “Sensor frame” to camera_link.
Set “Object frame” to handeye_target.
Set “End-effector frame” to panda_hand
Set “Robot base frame” to panda_link_0

Add, /handeye_calibration/target_detection

When the target is visible in the arm camera, and the axis is rendered on the target in the target detection image, you are ready to take your first calibration sample (pose pair).

Click the “Take sample” button in the “Manual calibration” section, and a new sample will be added to the “Pose samples” list on the left side of the panel.

Next, you can move the arm to a new pose using the “MotionPlanning” panel, or use your robot’s teaching pendant or free drive mode, if it has one, and click “Take sample” again.

Be sure to include some rotation between each pair of poses, and don’t always rotate around the same axis–at least two rotation axes are needed to uniquely solve for the calibration.

Once you have collected five samples, a calibration will be performed automatically, and updated each time a new sample is added.

The calibration will improve significantly with a few more samples, and will typically plateau after about 12 or 15 samples.

The position and orientation will be displayed on the “Context” tab, as mentioned above, and the published TF will be updated as well.

Click “Save camera pose” to export the calibration result. This will create a launch file with a static transform publisher containing the calibrated camera transform.
