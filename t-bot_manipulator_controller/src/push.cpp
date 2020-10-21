#include "../include/t-bot_manipulator_controller/Controller.h"

#include <ros/ros.h>
 
int main(int argc, char** argv)
{   
    ros::init(argc, argv, "push_node_controller");

    ROS_INFO("push_node_controller");

    push_button::PushButton pb;

    ros::Rate loop_rate(10);

    ros::Duration(1.0).sleep();
    pb.actuatorTorque(true);
    // pb.setInitPose();
    
    while (ros::ok())
    {
        pb.update();
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}