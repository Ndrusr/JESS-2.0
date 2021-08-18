#include<ros/ros.h>
#include<std_msgs/Float32.h>
#include<adele_control_2/adeleHWInterface.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "single_joint_command");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(3);

    

    ros::Publisher sweepPub = nh.advertise<std_msgs::Float32>("AdeleHW/single_joint_command", 1000);
    adele_control_2::AdeleHW singleJoint(nh, NULL);

    if(!singleJoint.initializeHardware()){
        ROS_ERROR_STREAM("Hardware Initialization FAILED!");
        ROS_ERROR_STREAM("Terminating Program");
        ros::shutdown();
        return 1;
    }

    spinner.start();
    singleJoint.directCommandWrite(1, -0.785);
    singleJoint.writeCommandsToHardware();
    std_msgs::Float32 commandMsg;
    commandMsg.data = static_cast<float>(singleJoint.directCommandAccess(1));
    sweepPub.publish(commandMsg);
    sleep(3);

    singleJoint.directCommandWrite(1, 0.785);
    singleJoint.writeCommandsToHardware();
    commandMsg.data = static_cast<float>(singleJoint.directCommandAccess(1));
    sweepPub.publish(commandMsg);
    sleep(3);

    spinner.stop();
    ROS_INFO_STREAM("Sweep complete, terminating program");
    return 0;

}