#ifndef ADELE_CONTROL_2_ADELEHWINTERFACE
#define ADELE_CONTROL_2_ADELEHWINTERFACE

#include <ros/ros.h>
#include <urdf/model.h>


#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <std_msgs/Float64.h>


#include<transmission_interface/transmission_interface_loader.h>
#include<array>
#include<vector>
#include<boost/shared_ptr.hpp>
#include<string>

//This code has largely been repurposed from the ros_control_boilerplate generic_hw_interface script
//LINK: https://github.com/PickNikRobotics/ros_control_boilerplate

namespace adele_control_2{
class AdeleHW: public hardware_interface::RobotHW{
public:
    AdeleHW();
    AdeleHW(const ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);
    virtual ~AdeleHW(){
    }

    bool initializeHardware();
    void setupListeners();

    void updateJointsFromHardware();
    void writeCommandsToHardware();

    void update(const ros::TimerEvent& ev);
    void read(ros::Duration& elapsed_time);
    void write(ros::Duration& elapsed_time);

    void read(ros::Time& time, ros::Duration& period);
    void write(ros::Time& time, ros::Duration& period);
    //void reset();


    virtual bool checkForConflict(...) const;

private:
    void registerActuatorInterfaces();

    bool loadTransmissions();

    hardware_interface::ActuatorStateInterface act_state_interface_;
    hardware_interface::PositionActuatorInterface pos_act_interface_;

    transmission_interface::RobotTransmissions transmissions_;
    std::unique_ptr <transmission_interface::TransmissionInterfaceLoader> transmission_loader_;
    transmission_interface::ActuatorToJointStateInterface* act_to_jnt_state_;
    transmission_interface::JointToActuatorPositionInterface* p_jnt_to_act_pos_;
    
    struct JointWithPos{
        double position;
        double velocity;
        double effort;
        double command;
        hardware_interface::ActuatorStateHandle stateHandle;
        hardware_interface::ActuatorHandle handle;
        double jointPos;

        JointWithPos() : 
         position(0), velocity(0), effort(0), command(0)
        {}

        JointWithPos(double pos):
            position(pos), velocity(0), effort(0), command(0)
        {}
    };
    JointWithPos actuators[6];
    
    std::vector<std::string> joint_names_;
    std::vector<std::string> actuator_names_;

    double homePose[6] = {0, 1.57, 1.57, 1.567, -1.57, 0};

    template<std::size_t N> void readOutput(const std_msgs::Float64 msg);


protected:
    std::string name_;

    virtual void loadURDF(const ros::NodeHandle& nh, std::string param_name);
    ros::NodeHandle nh_;
    urdf::Model* urdf_model_;

    ros::Publisher trajPublisher;

    // ros::Timer my_control_loop;
    ros::Duration elapsed_time;
    double loopHz;
    boost::shared_ptr<controller_manager::ControllerManager> controllerManager;
    std::array<ros::Subscriber, 6> trajSub;
};
}
#endif 
