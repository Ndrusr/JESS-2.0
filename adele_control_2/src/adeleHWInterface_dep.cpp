#include"ros_control_boilerplate/generic_hw_interface.h"
#include<limits.h>
#include<transmission_interface/transmission_interface_loader.h>
#include<ros/console.h>
#include<vector>

#include<rosparam_shortcuts/rosparam_shortcuts.h>



namespace ti= transmission_interface;


class AdeleHW : public ros_control_boilerplate::GenericHWInterface{
public:
    AdeleHW();

    AdeleHW(const ros::NodeHandle& nh, urdf::Model* urdf_model)
    {
        name_ = "adeleHW";
        nh_=nh;
        use_rosparam_joint_limits_= false;
        use_soft_limits_if_available_= false;
        // Check if the URDF model needs to be loaded
        if (urdf_model == NULL)
            loadURDF(nh, "robot_description");
        else
            urdf_model_ = urdf_model;

        // Load rosparams
        ros::NodeHandle rpnh(
            nh_, "hardware_interface");  // TODO(davetcoleman): change the namespace to "generic_hw_interface" aka name_
        std::size_t error = 0;
        error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
        rosparam_shortcuts::shutdownIfError(name_, error);
        error = 0;
        error += !rosparam_shortcuts::get(name_, rpnh, "actuators", actuator_names);
        rosparam_shortcuts::shutdownIfError(name_, error);

        init();

        
    }
    void init(){
        num_joints_ = joint_names_.size();
        num_actuators_ = actuator_names.size();
        // Status
        joint_position_.resize(num_joints_, 0.0);
        joint_velocity_.resize(num_joints_, 0.0);
        joint_effort_.resize(num_joints_, 0.0);

        // Command
        joint_position_command_.resize(num_joints_, 0.0);
        joint_velocity_command_.resize(num_joints_, 0.0);
        joint_effort_command_.resize(num_joints_, 0.0);

        // Limits
        joint_position_lower_limits_.resize(num_joints_, 0.0);
        joint_position_upper_limits_.resize(num_joints_, 0.0);
        joint_velocity_limits_.resize(num_joints_, 0.0);
        joint_effort_limits_.resize(num_joints_, 0.0);

        // Initialize interfaces for each joint //Disabled to initialize interfaces for actuators instead
        // for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
        // {
        //     ROS_DEBUG_STREAM_NAMED(name_, "Loading joint name: " << joint_names_[joint_id]);

        //     // Create joint state interface
        //     joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
        //         joint_names_[joint_id], &joint_position_[joint_id], &joint_velocity_[joint_id], &joint_effort_[joint_id]));

        //     // Add command interfaces to joints
        //     // TODO: decide based on transmissions?
        //     hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(
        //         joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_position_command_[joint_id]);
        //     position_joint_interface_.registerHandle(joint_handle_position);

        //     hardware_interface::JointHandle joint_handle_velocity = hardware_interface::JointHandle(
        //         joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_velocity_command_[joint_id]);
        //     velocity_joint_interface_.registerHandle(joint_handle_velocity);

        //     hardware_interface::JointHandle joint_handle_effort = hardware_interface::JointHandle(
        //         joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_effort_command_[joint_id]);
        //     effort_joint_interface_.registerHandle(joint_handle_effort);

        //     // Load the joint limits
        //     registerJointLimits(joint_handle_position, joint_handle_velocity, joint_handle_effort, joint_id);
        // }  // end for each joint

        // registerInterface(&joint_state_interface_);     // From RobotHW base class.
        // registerInterface(&position_joint_interface_);  // From RobotHW base class.
        // registerInterface(&velocity_joint_interface_);  // From RobotHW base class.
        // registerInterface(&effort_joint_interface_);    // From RobotHW base class.
        
        ROS_INFO_STREAM("Preparing actuator state interface...");

        for(std::size_t actuator_id = 0; actuator_id < num_actuators_; actuator_id++){
            
        }

        ROS_INFO_STREAM_NAMED(name_, "AdeleHW Ready.");
    }

protected:
    ti::RobotTransmissions robot_transmissions_;
    std::unique_ptr<ti::TransmissionInterfaceLoader> transmission_loader_;

    ti::ActuatorToJointStateInterface* actToJntState;
    ti::JointToActuatorPositionInterface* jntToActPos;

    std::size_t num_actuators_;
    std::vector<std::string> actuator_names;




};


