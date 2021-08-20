#include<ros/ros.h>
#include<std_msgs/Float32.h>
#include<adele_control_2/adeleHWInterface.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "single_joint_command");
    ros::NodeHandle nh;
    std::string xml, xml_add;
    ros::AsyncSpinner spinner(3);

    // if(nh.searchParam("dummy_xml", xml_add)){
    //     nh.getParam(xml_add, xml);
    //     ROS_INFO_STREAM("Sample xml string: " << xml);
    // }else{
    //     ROS_INFO_STREAM("Cannot find debug parameter");
    // }
    

    ros::Publisher sweepPub = nh.advertise<std_msgs::Float32>("AdeleHW/single_joint_command", 1000);
    adele_control_2::AdeleHW singleJoint(nh, NULL);
    
    if(!singleJoint.initializeHardware()){
        ROS_ERROR_STREAM("Hardware Initialization FAILED!");
        ROS_ERROR_STREAM("Terminating Program");
        ros::shutdown();
        return 1;
    }else{
        ROS_INFO_STREAM("Hardware initialised successfully!");
    }

    spinner.start();
    ROS_INFO_STREAM("Spinner started.");
    singleJoint.directCommandWrite(1, -0.785);
    ROS_INFO_STREAM("Writing command for -0.785 position");
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