#include<time.h>
#include<controller_manager/controller_manager.h>
#include<hardware_interface/hardware_interface.h>

namespace adele_control_2{
static const double BILLION = 1000000000.0;

class AdeleHWControlLoop
{

public:
    AdeleHWControlLoop(ros::NodeHandle& nh, std::shared_ptr<hardware_interface::RobotHW> hardwareInterface);
    void run();
protected:
    void update();
    
    ros::NodeHandle nh_;
    std::string name_ = "AdeleControlLoop";

    ros::Duration desired_update_period;
    double cycle_time_error_threshold;
    
    ros::Duration elapsed_time;
    double loopHZ;
    struct timespec lastTime;
    struct timespec currentTime;

    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    std::shared_ptr<hardware_interface::RobotHW> hardware_interface_;
};


    
}