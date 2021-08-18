#include<adele_control_2/adeleHWInterface.h>
#include<ros/console.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include<control_msgs/FollowJointTrajectoryAction.h>
#include<trajectory_msgs/JointTrajectory.h>

//This code has largely been repurposed from the ros_control_boilerplate generic_hw_interface script
//LINK: https://github.com/PickNikRobotics/ros_control_boilerplate

namespace adele_control_2
{

AdeleHW::AdeleHW(){
    //default constructor, do not use
}

AdeleHW::AdeleHW(const ros::NodeHandle& nh, urdf::Model* urdf_model):
    nh_(nh), urdf_model_(urdf_model), name_("adele_hw_interface")
{
    
    

    if (urdf_model == NULL)
        loadURDF(nh, "/robot_description");
    else
        urdf_model_ = urdf_model;

  // Load rosparams
    
    ros::NodeHandle rpnh(
      nh_, name_);

    controllerManager.reset(new controller_manager::ControllerManager(this, nh_));
    
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(name_, rpnh, "actuators", actuator_names_);
    error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
    rosparam_shortcuts::shutdownIfError(name_, error);

    directControl = true;

    //ros::ServiceClient jntTraj = nh_.serviceClient<control_msgs::FollowJointTrajectoryAction>("followTraj");
    ros::Publisher trajPublisher = nh_.advertise<trajectory_msgs::JointTrajectoryPoint> ("AdeleHW/real_actuator_trajectory", 1000);

    
}

AdeleHW::~AdeleHW() {
    delete(urdf_model_);
}  

bool AdeleHW::loadTransmissions(){
        using namespace transmission_interface;
  // Initialize transmission loader
        try
        {
            transmission_loader_.reset(new TransmissionInterfaceLoader(this, &transmissions_));
        }
        catch(const std::invalid_argument& ex)
        {
            ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
            return false;
        }
        catch(const pluginlib::LibraryLoadException& ex)
        {
            ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
            return false;
        }
        catch(...)
        {
            ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
            return false;
        } 

  // Parse robot description
        const std::string model_param_name = "robot_description";
        bool res = nh_.hasParam(model_param_name);
        std::string robot_description = ""; 
        if (!res || !nh_.getParam(model_param_name, robot_description))
        {
            ROS_ERROR("Robot descripion couldn't be retrieved from param server.");
            return false;
        }
        

  // Perform actual transmission loading
        if (!transmission_loader_->load(robot_description)) {return false;}
        ROS_INFO("Loaded transmissions from URDF");

  // Get the transmission interfaces
        act_to_jnt_state_ = transmissions_.get<ActuatorToJointStateInterface>();
        p_jnt_to_act_pos_ = transmissions_.get<JointToActuatorPositionInterface>();
        return true;
}


void AdeleHW::registerActuatorInterfaces(){
    std::size_t numActuators = sizeof(actuators);
    using namespace hardware_interface;
    for (std::size_t i = 0; i < numActuators; i++){
        ActuatorStateHandle act_state_handle(actuator_names_[i], 
            &actuators[i].position, &actuators[i].velocity, &actuators[i].effort);
        actuators[i].stateHandle = act_state_handle;
        actuators[i].handle = ActuatorHandle(actuators[i].stateHandle, &actuators[i].command);
        act_state_interface_.registerHandle( actuators[i].stateHandle );
        pos_act_interface_.registerHandle(actuators[i].handle);
    }

    registerInterface(&act_state_interface_);
    registerInterface(&pos_act_interface_);

}

bool AdeleHW::initializeHardware(){
// Register interfaces with the RobotHW interface manager, allowing ros_control operation
  registerActuatorInterfaces();
  // Load transmission information from URDF
  if(!loadTransmissions()){return false;}
  return true;
}

void AdeleHW::loadURDF(const ros::NodeHandle& nh, std::string param_name){
    urdf_model_ = new urdf::Model();
    bool loaded = false;
    std::string urdf_string;
    // search and wait for robot_description on param server
    while (urdf_string.empty() && ros::ok())
    {
    
    std::string search_param_name;
    if (nh.searchParam(param_name, search_param_name))
    {
        ROS_INFO_STREAM_NAMED(name_, "Parameter found. Waiting for model URDF on the ROS param server at location: " << nh.getNamespace()
                                                                                                    << search_param_name);
        //I DON'T GET IT, WHY IS THE F***ING PARAMETER SERVER RETURNING A NULLPTR WHEN A STRING IS CLEARLY STORED THERE
        loaded = nh.getParam(search_param_name, urdf_string);
    }
    else
    {
        ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " << nh.getNamespace()
                                                                                                    << param_name);
        loaded = nh.getParam(search_param_name, urdf_string);
        
    }
    
    if(!loaded)
        ROS_INFO_STREAM("Failed to get parameter, trying again...");
    usleep(100000);
} //while

    if (!urdf_model_->initString(urdf_string))
        ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
    else
        ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");

} //loadURDF

/*
void AdeleHW::reset()
{
  // Reset joint limits state, in case of mode switch or e-stop
}
*/

void AdeleHW::setupListeners(){
    size_t num_joints = joint_names_.size();
    std::string prefix = "AdeleHW/act";
    std::string suffix = "State";
    for(size_t count = 0; count != num_joints; count++){
        std::string temp = prefix + std::to_string(count) + suffix;
        //trajSub[count] = nh_.subscribe(temp, 1000,);
    }
}

void AdeleHW::updateJointsFromHardware(){
    //TODO: provide script to read from MCU
    act_to_jnt_state_->propagate();
    size_t num_joints = joint_names_.size();
    double posTmp = 0;
    for(size_t count = 0; count != num_joints; count++){
        posTmp = this->get<hardware_interface::JointStateInterface>()->getHandle(joint_names_[count]).getPosition();
        actuators[count].jointPos = posTmp;
        
    }

}

void AdeleHW::writeCommandsToHardware(){
    p_jnt_to_act_pos_->propagate();
    
    //TODO: provide script to write command
    
}

void AdeleHW::read(ros::Time& time, ros::Duration& period){
    ros::Duration elapsed_time = period;
    read(elapsed_time);
}

void AdeleHW::write(ros::Time& time, ros::Duration& period){
    ros::Duration elapsed_time = period;
    write(elapsed_time);
}

void AdeleHW::read(ros::Duration& elapsed_time){
    size_t num_joints = joint_names_.size();
    updateJointsFromHardware();
    //trajPublisher.publish()
}

void AdeleHW::write(ros::Duration& elapsed_time){
    writeCommandsToHardware();
    size_t num_joints = joint_names_.size();
    double commandTmp = 0.0;

    trajectory_msgs::JointTrajectoryPoint trajGoal;

    for(size_t count = num_joints; count != -1; count -= 1){
        commandTmp = actuators[count].command;
        trajGoal.positions.push_back(commandTmp);
        trajGoal.effort.push_back(actuators[count].effort);
        trajGoal.velocities.push_back(actuators[count].velocity);
        trajGoal.accelerations.push_back(0.0);
    }
    

    trajPublisher.publish(trajGoal);
}
void AdeleHW::directCommandWrite(int linkNo, double commandValue){
    if(directControl){
        this->get<hardware_interface::JointCommandInterface>()->getHandle(joint_names_[linkNo]).setCommand(commandValue);
    }    
    else{
        ROS_ERROR_STREAM("Direct Control has NOT been enabled!");
    }
}

double AdeleHW::directCommandAccess(int linkNo){
    if(directControl){
        return actuators[linkNo].command;
    }
    else{
        ROS_ERROR_STREAM("Direct Control has NOT been enabled!");
        return 0;
    }
}

bool AdeleHW::checkForConflict(...) const {
    return false;
}

}
/*
void AdeleHW::update(const ros::TimerEvent& ev){
    elapsed_time = ros::Duration(ev.current_real-ev.last_real);
    read(elapsed_time);
    controllerManager->update(ros::Time::now(), elapsed_time);
    write(elapsed_time);
}
*/
