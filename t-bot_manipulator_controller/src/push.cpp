#include "../include/t-bot_manipulator_controller/Controller.h"

#include <ros/ros.h>
 
int main(int argc, char** argv)
{   
    // bool a;
    ros::init(argc, argv, "push_node_controller");

    ROS_INFO("push_node_controller");

    push_button::PushButton pb;

    // pb.update();

    // ros::Rate loop_rate(10);

    // ros::Duration(1.0).sleep();
    // pb.actuatorTorque(true);
    
    // // // pb.update();
    // while (ros::ok())
    // {
    //     pb.update();
    //     // ros::spinOnce();
    //     // ros::spinOnce();
    //     loop_rate.sleep();
    //     ros::spinOnce();
    // }
    // ros::spinOnce();
    return 0;
}