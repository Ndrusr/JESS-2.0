#include<adele_control_2/adeleControlLoop.h>

#include<rosparam_shortcuts/rosparam_shortcuts.h>

//This code has largely been repurposed from the ros_control_boilerplate generic_hw_control_loop script
//LINK: https://github.com/PickNikRobotics/ros_control_boilerplate

namespace adele_control_2{

AdeleHWControlLoop::AdeleHWControlLoop(ros::NodeHandle& nh, 
                                        std::shared_ptr<hardware_interface::RobotHW> hardwareInterface)
                                        :nh_(nh), hardware_interface_(hardwareInterface)
{
    // Create the controller manager
  controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(), nh_));

  // Load rosparams
  ros::NodeHandle rpsnh(nh, name_);
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, rpsnh, "loop_hz", loopHZ);
  error += !rosparam_shortcuts::get(name_, rpsnh, "cycle_time_error_threshold", cycle_time_error_threshold);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Get current time for use with first update
  clock_gettime(CLOCK_MONOTONIC, &lastTime);

  desired_update_period = ros::Duration(1 / loopHZ);
}

void AdeleHWControlLoop::run(){
    ros::Rate rate(loopHZ);
    while (ros::ok())
    {
        update();
        rate.sleep();
    }
}

void AdeleHWControlLoop::update()
{
  // Get change in time
  clock_gettime(CLOCK_MONOTONIC, &currentTime);
  elapsed_time =
      ros::Duration(currentTime.tv_sec - lastTime.tv_sec + (currentTime.tv_nsec - lastTime.tv_nsec) / BILLION);
  lastTime = currentTime;
  ros::Time now = ros::Time::now();
  // ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "generic_hw_main","Sampled update loop
  // with elapsed time " << elapsed_time_.toSec());

  // Error check cycle time
  const double cycle_time_error = (elapsed_time - desired_update_period).toSec();
  if (cycle_time_error > cycle_time_error_threshold)
  {
    ROS_WARN_STREAM_NAMED(name_, "Cycle time exceeded error threshold by: "
                                     << cycle_time_error << ", cycle time: " << elapsed_time
                                     << ", threshold: " << cycle_time_error_threshold);
  }

  // Input
  hardware_interface_->read(now, elapsed_time);

  // Control
  controller_manager_->update(now, elapsed_time);

  // Output
  hardware_interface_->write(now, elapsed_time);
}

}
