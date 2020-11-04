/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include "../../include/t-bot_manipulator_controller/Controller.h"

#include <math.h>

#define LIFT_UP         "("
#define LIFT_DOWN       ")"
#define STARTING_POINT  "B1"  
namespace push_button
{
PushButton::PushButton()
{
    ros::NodeHandle nh_("");
    push_start = false;
    is_triggered = false;
    open_manipulator_moveing_state = "STOPPED"; 
    
    set_joint_position = nh_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
    set_kinematics_position = nh_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_position_only");
    set_joint_position_from_present = nh_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path_from_present");
    set_actuator_state = nh_.serviceClient<open_manipulator_msgs::SetActuatorState>("set_actuator_state");
    set_gripper_control = nh_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");

    open_manipulator_joint_state_sub = nh_.subscribe("/joint_states", 10, &PushButton::jointStatesCallback, this);
    open_manipulator_kinematics_pose_sub = nh_.subscribe("/gripper/kinematics_pose", 10, &PushButton::kinematicsPoseCallback, this);
    open_manipulator_states_sub = nh_.subscribe("/states", 10, &PushButton::statesCallback, this);
    marker_point_sub = nh_.subscribe("/button_tracker_3d/markers", 10, &PushButton::markerCallback, this);
    floor_sub = nh_.subscribe("/floor", 10, &PushButton::floorCallback, this);

    button = LIFT_UP;
}

PushButton::~PushButton()
{
    if (is_shutdown == true)
        std::cout << "program shutdown..." << std::endl;
}

void PushButton::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr& msg)
{
    std::vector<double> temp_position;

    // currentToolPose = msg->pose;
    // ROS_WARN("  currentToolPose x,y,z  %.2f, %.2f, %.2f  ", currentToolPose.position.x, currentToolPose.position.y, currentToolPose.position.z);

    temp_position.push_back(msg->pose.position.x);
    temp_position.push_back(msg->pose.position.y);
    temp_position.push_back(msg->pose.position.z);

    kinematicsStates = temp_position;

    // kinematics_pose_.pose = temp_position;

    // kinematicsStates[0] = msg->pose.position.x;
    // kinematicsStates[1] = msg->pose.position.y;
    // kinematicsStates[2] = msg->pose.position.z;
}

void PushButton::markerCallback(const button_recognition_msgs::MarkerArray::ConstPtr& msg)
{
    if (!msg->markers.empty())
    {
        stack_size = msg->markers.size();
        for (int i = 0; i < stack_size; i++)
        {
            push_start = true;
            pushButtonPose.header = msg->markers[i].header;
            pushButtonPose.pose = msg->markers[i].pose;
            // button_floor = msg->markers[i].Class;
            // button_floor.push_back(msg->markers[i].Class);

            marker.push_back(msg->markers[i]);
            // std::cout << "class : " << msg->markers[i].Class << std::endl;
            // marker_array.markers.push_back(msg->markers[i]);
        }
    }
    else
    {
        push_start = false;
    }
}

void PushButton::floorCallback(const std_msgs::String::ConstPtr& msg)
{
    goal_button = msg->data.substr(18, 1);
}

void PushButton::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    is_triggered = true;

    for (int i = 0; i < msg->position.size(); i++)
    {
        jointState.push_back(msg->position[i]);
    }
}

void PushButton::statesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr& msg)
{
    open_manipulator_moveing_state = msg->open_manipulator_moving_state;
}

bool PushButton::setInitPose()
{
    ROS_WARN("setInitPose");

    open_manipulator_msgs::SetJointPosition srv;
    srv.request.joint_position.joint_name.push_back("joint1");
    srv.request.joint_position.joint_name.push_back("joint2");
    srv.request.joint_position.joint_name.push_back("joint3");
    srv.request.joint_position.joint_name.push_back("joint4");
    srv.request.joint_position.position.push_back(0.0);
    srv.request.joint_position.position.push_back(-0.95);
    srv.request.joint_position.position.push_back(0.32);
    srv.request.joint_position.position.push_back(0.65);

    bool resp = false;
    try
    {
        srv.request.path_time = 2.0;
        resp = set_joint_position.call(srv);
        ros::Duration(srv.request.path_time).sleep();
    }
    catch(const ros::Exception& e)
    {
        std::cout << "Service call failed: " << &e << std::endl;
    }
    
    if (resp != true)
    {
    }
}

bool PushButton::setBackwardPose()
{
    ROS_WARN("setInitPose");

    // init position
    open_manipulator_msgs::SetJointPosition srv;
    srv.request.joint_position.joint_name.push_back("joint1");
    srv.request.joint_position.joint_name.push_back("joint2");
    srv.request.joint_position.joint_name.push_back("joint3");
    srv.request.joint_position.joint_name.push_back("joint4");
    srv.request.joint_position.position.push_back(0.0);
    srv.request.joint_position.position.push_back(-1.791);
    srv.request.joint_position.position.push_back(0.507);
    srv.request.joint_position.position.push_back(1.438);
    srv.request.planning_group = "arm";
    
    bool resp_joint = false;
    try
    {
        srv.request.path_time = 2.0;
        resp_joint = set_gripper_control.call(srv);
        ros::Duration(srv.request.path_time).sleep();
    }
    catch(const ros::Exception& e)
    {
        std::cout << "Service call failed: " << &e << std::endl;
    }
    
    if (resp_joint != true)
    {
        return false;
    }

    open_manipulator_msgs::JointPosition gripper_position;
    open_manipulator_msgs::SetJointPosition srv_gripper;
    srv_gripper.request.joint_position.joint_name.push_back("gripper");
    srv_gripper.request.joint_position.position.push_back(0.01);
    srv_gripper.request.planning_group = "gripper";

    bool resp_gripper = false;
    try
    {
        srv_gripper.request.path_time  = 2.0;
        resp_gripper = set_gripper_control.call(srv_gripper);
        ros::Duration(srv_gripper.request.path_time).sleep();
    }
    catch(const ros::Exception& e)
    {
        std::cout << "Service call failed: " << &e << std::endl;
    }
    
    if (resp_gripper != true)
    {
        return false;
    }

    return true;
}

bool PushButton::setBackwardPose2()
{
    ROS_WARN("setInitPose");

    // init position
    open_manipulator_msgs::SetJointPosition srv;
    srv.request.joint_position.joint_name.push_back("joint1");
    srv.request.joint_position.joint_name.push_back("joint2");
    srv.request.joint_position.joint_name.push_back("joint3");
    srv.request.joint_position.joint_name.push_back("joint4");
    srv.request.joint_position.position.push_back(0.0);
    srv.request.joint_position.position.push_back(-1.791);
    srv.request.joint_position.position.push_back(0.507);
    srv.request.joint_position.position.push_back(1.438);
    srv.request.planning_group = "arm";
    
    bool resp_joint = false;
    try
    {
        srv.request.path_time = 2.0;
        resp_joint = set_gripper_control.call(srv);
        ros::Duration(srv.request.path_time).sleep();
    }
    catch(const ros::Exception& e)
    {
        std::cout << "Service call failed: " << &e << std::endl;
    }
    
    if (resp_joint != true)
    {
        return false;
    }

    return true;
}

bool PushButton::actuatorTorque(bool enable)
{
    ROS_WARN("actuatorTorque");
    bool resp = false;

    open_manipulator_msgs::SetActuatorState srv;
    srv.request.set_actuator_state = enable;

    // std::vector<std::string> joint_name;
    // joint_name.push_back("joint1");
    // joint_name.push_back("joint2");
    // joint_name.push_back("joint3");
    // joint_name.push_back("joint4");
    // joint_name.push_back("gripper");

    try
    {
        resp = set_actuator_state.call(srv);
        ros::Duration(1).sleep();
    }
    catch(const ros::Exception& e)
    {
        std::cout << "Service call failed: " << &e << std::endl;
        return false;
    }

    if (resp != true)
    {
        ROS_INFO("set_actuator enable fail");
    }
    return resp;
}

geometry_msgs::Point PushButton::forwardButtonPosition(geometry_msgs::Point button_position, double forward_distance)
{
    geometry_msgs::Point resultPoint;
    
    if (std::abs(button_position.x) < 0.001)
    {
        button_position.x = 0.001;
    }

    double radian = atan(button_position.y / button_position.x);
    double degree = radian * (180.0 / 3.14);
    double dist = forward_distance;
    double distX = cos(radian) * dist;
    double distY = sin(radian) * dist;

    resultPoint.x = button_position.x + distX;
    resultPoint.y = button_position.y + distY;
    resultPoint.z = button_position.z;

    ROS_INFO("%.2f m forward,so objectposition change xyz(%.2f ,%.2f, %.2f) -> xyz(%.2f ,%.2f, %.2f)", 
            forward_distance, button_position.x, button_position.y, button_position.z, 
            resultPoint.x, resultPoint.y, resultPoint.z);

    return resultPoint;
}

bool PushButton::moveToUpButton()
{
    bool resp = false;

    if (pushButtonPose.pose.position.x > 0)
    {   
        ROS_WARN("move to button");

        for (int i = 0; i < stack_size; i++)
        {   
            // std::cout << target << std::endl;
            if (button_floor[i] == LIFT_UP)
            {
                open_manipulator_msgs::SetKinematicsPose srv;
                srv.request.end_effector_name = "gripper";
                srv.request.planning_group = "arm";
                srv.request.kinematics_pose.pose = pushButtonPose.pose;
                // kinematics_pose.pose = pushButtonPose.pose;

                ROS_INFO_STREAM(pushButtonPose.header);
                ROS_INFO_STREAM(pushButtonPose.pose);

                srv.request.kinematics_pose.pose.position = forwardButtonPosition(srv.request.kinematics_pose.pose.position, 0.05);
                // srv.request.kinematics_pose.pose.position.x += 0.01;
                srv.request.kinematics_pose.pose.position.y -= 0.015;
                // srv.request.kinematics_pose.pose.position.z += 0.015;

                double moveDistance = sqrt(pow((srv.request.kinematics_pose.pose.position.x - currentToolPose.position.x), 2)
                                        +  pow((srv.request.kinematics_pose.pose.position.y - currentToolPose.position.y), 2)
                                        +  pow((srv.request.kinematics_pose.pose.position.z - currentToolPose.position.z), 2));

                srv.request.path_time = moveDistance * 10.0;
                double operating_time = srv.request.path_time;
                double operating_limit_time = operating_time;

                if (operating_time < 1.0)
                {
                    operating_limit_time = 1.0;
                }
                else if (operating_time > 3.0)
                {
                    operating_limit_time = 3.0;
                }

                ROS_WARN("go xyz %.2f,%.2f,%.2f , moveDistance %.2f, operate time %.2f ( %.2f )", \
                        srv.request.kinematics_pose.pose.position.x, srv.request.kinematics_pose.pose.position.y, srv.request.kinematics_pose.pose.position.z, \
                        moveDistance, operating_time, operating_limit_time);

                try
                {
                    resp = set_kinematics_position.call(srv);
                    button = goal_button;
                }
                catch(const ros::Exception& e)
                {
                    std::cout << "Service call failed: " << &e << std::endl;
                    return false;
                }
            }
        }
    }
    else
    {
        ROS_INFO("Cannot go to the position of the button");
        return resp;
    }
}

bool PushButton::moveToGoalButton()
{
    bool resp = false;

    if (pushButtonPose.pose.position.x > 0)
    {   
        ROS_WARN("move to button");

        for (auto target : button_floor)
        {
            if (target == goal_button)
            {
                open_manipulator_msgs::SetKinematicsPose srv;
                srv.request.end_effector_name = "gripper";
                srv.request.planning_group = "arm";
                srv.request.kinematics_pose.pose = pushButtonPose.pose;
                // kinematics_pose.pose = pushButtonPose.pose;

                ROS_INFO_STREAM(pushButtonPose.header);
                ROS_INFO_STREAM(pushButtonPose.pose);

                srv.request.kinematics_pose.pose.position = forwardButtonPosition(srv.request.kinematics_pose.pose.position, 0.05);
                srv.request.kinematics_pose.pose.position.x -= 0.01;
                srv.request.kinematics_pose.pose.position.y -= 0.015;
                srv.request.kinematics_pose.pose.position.z -= 0.015;

                double moveDistance = sqrt(pow((srv.request.kinematics_pose.pose.position.x - currentToolPose.position.x), 2)
                                        +  pow((srv.request.kinematics_pose.pose.position.y - currentToolPose.position.y), 2)
                                        +  pow((srv.request.kinematics_pose.pose.position.z - currentToolPose.position.z), 2));

                srv.request.path_time = moveDistance * 10.0;
                double operating_time = srv.request.path_time;
                double operating_limit_time = operating_time;

                if (operating_time < 1.0)
                {
                    operating_limit_time = 1.0;
                }
                else if (operating_time > 3.0)
                {
                    operating_limit_time = 3.0;
                }

                ROS_WARN("go xyz %.2f,%.2f,%.2f , moveDistance %.2f, operate time %.2f ( %.2f )", \
                        srv.request.kinematics_pose.pose.position.x, srv.request.kinematics_pose.pose.position.y, srv.request.kinematics_pose.pose.position.z, \
                        moveDistance, operating_time, operating_limit_time);

                try
                {
                    resp = set_kinematics_position.call(srv);
                    button = STARTING_POINT;
                }
                catch(const ros::Exception& e)
                {
                    std::cout << "Service call failed: " << &e << std::endl;
                    return false;
                }

            }
        }
    }
    else
    {
        ROS_INFO("Cannot go to the position of the button");
        return resp;
    }
}

bool PushButton::moveToStartPointButton()
{
    bool resp = false;

    if (pushButtonPose.pose.position.x > 0)
    {   
        ROS_WARN("move to button");

        for (auto target : button_floor)
        {
            if (target == STARTING_POINT)
            {
                open_manipulator_msgs::SetKinematicsPose srv;
                srv.request.end_effector_name = "gripper";
                srv.request.planning_group = "arm";
                srv.request.kinematics_pose.pose = pushButtonPose.pose;
                // kinematics_pose.pose = pushButtonPose.pose;

                ROS_INFO_STREAM(pushButtonPose.header);
                ROS_INFO_STREAM(pushButtonPose.pose);

                srv.request.kinematics_pose.pose.position = forwardButtonPosition(srv.request.kinematics_pose.pose.position, 0.05);
                // srv.request.kinematics_pose.pose.position.x += 0.01;
                srv.request.kinematics_pose.pose.position.y -= 0.015;
                srv.request.kinematics_pose.pose.position.z += 0.015;

                double moveDistance = sqrt(pow((srv.request.kinematics_pose.pose.position.x - currentToolPose.position.x), 2)
                                        +  pow((srv.request.kinematics_pose.pose.position.y - currentToolPose.position.y), 2)
                                        +  pow((srv.request.kinematics_pose.pose.position.z - currentToolPose.position.z), 2));

                srv.request.path_time = moveDistance * 10.0;
                double operating_time = srv.request.path_time;
                double operating_limit_time = operating_time;

                if (operating_time < 1.0)
                {
                    operating_limit_time = 1.0;
                }
                else if (operating_time > 3.0)
                {
                    operating_limit_time = 3.0;
                }

                ROS_WARN("go xyz %.2f,%.2f,%.2f , moveDistance %.2f, operate time %.2f ( %.2f )", \
                        srv.request.kinematics_pose.pose.position.x, srv.request.kinematics_pose.pose.position.y, srv.request.kinematics_pose.pose.position.z, \
                        moveDistance, operating_time, operating_limit_time);

                try
                {
                    resp = set_kinematics_position.call(srv);
                    button = LIFT_DOWN;
                }
                catch(const ros::Exception& e)
                {
                    std::cout << "Service call failed: " << &e << std::endl;
                    return false;
                }
                
            }
        }
    }
    else
    {
        ROS_INFO("Cannot go to the position of the button");
        return resp;
    }
}

bool PushButton::moveToDownButton()
{
    bool resp = false;

    if (pushButtonPose.pose.position.x > 0)
    {   
        ROS_WARN("move to button");

        for (auto target : button_floor)
        {
            if (target == LIFT_DOWN)
            {
                open_manipulator_msgs::SetKinematicsPose srv;
                srv.request.end_effector_name = "gripper";
                srv.request.planning_group = "arm";
                srv.request.kinematics_pose.pose = pushButtonPose.pose;
                // kinematics_pose.pose = pushButtonPose.pose;

                ROS_INFO_STREAM(pushButtonPose.header);
                ROS_INFO_STREAM(pushButtonPose.pose);

                srv.request.kinematics_pose.pose.position = forwardButtonPosition(srv.request.kinematics_pose.pose.position, 0.05);
                // srv.request.kinematics_pose.pose.position.x += 0.01;
                srv.request.kinematics_pose.pose.position.y -= 0.015;
                srv.request.kinematics_pose.pose.position.z += 0.015;

                double moveDistance = sqrt(pow((srv.request.kinematics_pose.pose.position.x - currentToolPose.position.x), 2)
                                        +  pow((srv.request.kinematics_pose.pose.position.y - currentToolPose.position.y), 2)
                                        +  pow((srv.request.kinematics_pose.pose.position.z - currentToolPose.position.z), 2));

                srv.request.path_time = moveDistance * 10.0;
                double operating_time = srv.request.path_time;
                double operating_limit_time = operating_time;

                if (operating_time < 1.0)
                {
                    operating_limit_time = 1.0;
                }
                else if (operating_time > 3.0)
                {
                    operating_limit_time = 3.0;
                }

                ROS_WARN("go xyz %.2f,%.2f,%.2f , moveDistance %.2f, operate time %.2f ( %.2f )", \
                        srv.request.kinematics_pose.pose.position.x, srv.request.kinematics_pose.pose.position.y, srv.request.kinematics_pose.pose.position.z, \
                        moveDistance, operating_time, operating_limit_time);

                try
                {
                    resp = set_kinematics_position.call(srv);
                    button = LIFT_UP;
                }
                catch(const ros::Exception& e)
                {
                    std::cout << "Service call failed: " << &e << std::endl;
                    return false;
                }
                
            }
        }
    }
    else
    {
        ROS_INFO("Cannot go to the position of the button");
        return resp;
    }
}

bool PushButton::moveToButton(std::string target)
{
    bool resp = false;

    if (pushButtonPose.pose.position.x > 0)
    {   
        ROS_WARN("move to button");

        for (int i = 0; i < stack_size; i++)
        {
            std::cout << "target : " << target << std::endl;
            if (target == marker[i].Class)
            {
                open_manipulator_msgs::SetKinematicsPose srv;
                srv.request.end_effector_name = "gripper";
                srv.request.planning_group = "arm";
                srv.request.kinematics_pose.pose = marker[i].pose;
                // srv.request.kinematics_pose.pose = pushButtonPose.pose;
                // kinematics_pose.pose = pushButtonPose.pose;

                ROS_INFO_STREAM(pushButtonPose.header);
                ROS_INFO_STREAM(pushButtonPose.pose);

                srv.request.kinematics_pose.pose.position = forwardButtonPosition(srv.request.kinematics_pose.pose.position, 0.05);
                // srv.request.kinematics_pose.pose.position.x += 0.01;
                srv.request.kinematics_pose.pose.position.y -= 0.02;
                srv.request.kinematics_pose.pose.position.z -= 0.04;

                double moveDistance = sqrt(pow((srv.request.kinematics_pose.pose.position.x - currentToolPose.position.x), 2)
                                        +  pow((srv.request.kinematics_pose.pose.position.y - currentToolPose.position.y), 2)
                                        +  pow((srv.request.kinematics_pose.pose.position.z - currentToolPose.position.z), 2));

                srv.request.path_time = moveDistance * 10.0;
                double operating_time = srv.request.path_time;
                double operating_limit_time = operating_time;

                if (operating_time < 1.0)
                {
                    operating_limit_time = 1.0;
                }
                else if (operating_time > 3.0)
                {
                    operating_limit_time = 3.0;
                }

                ROS_WARN("go xyz %.2f,%.2f,%.2f , moveDistance %.2f, operate time %.2f ( %.2f )", \
                        srv.request.kinematics_pose.pose.position.x, srv.request.kinematics_pose.pose.position.y, srv.request.kinematics_pose.pose.position.z, \
                        moveDistance, operating_time, operating_limit_time);

                try
                {
                    resp = set_kinematics_position.call(srv);
                    return resp;
                }
                catch(const ros::Exception& e)
                {
                    std::cout << "Service call failed: " << &e << std::endl;
                    return false;
                }
            }
            else
            {
                continue;
            }
            
        }
    }
    else
    {
        ROS_INFO("Cannot go to the position of the button");
        return resp;
    }
}

bool PushButton::setButtonType()
{
    if (button == LIFT_UP)
    {
        ROS_INFO("move to up button node");
        if (moveToButton(button))
        {
            button = goal_button;    
            marker.erase(marker.begin(), marker.begin() + marker.size());
            return true;
        }
        else
        {
            return false;
        }
        
    }
    else if (button == goal_button)
    {
        ROS_INFO("move to goal button node");
        if (moveToButton(button))
        {
            button = STARTING_POINT;
            marker.erase(marker.begin(), marker.begin() + marker.size());
            return true;
        }
        else
        {
            return false;
        }
    }
    else if (button == STARTING_POINT)
    {
        ROS_INFO("move to starting point button node");
        if (moveToButton(button))
        {
            button = LIFT_DOWN;
            marker.erase(marker.begin(), marker.begin() + marker.size());
            return true;
        }
        else
        {
            return false;
        }
    }
    else if (button == LIFT_DOWN)
    {
        ROS_INFO("move to down button node");
        if (moveToButton(button))
        {
            button = LIFT_UP;
            marker.erase(marker.begin(), marker.begin() + marker.size());
            return true;
        }
        else
        {
            return false;
        }
    }
}

void PushButton::update()
{
    if (is_triggered == true)
    {
        // setInitPose();
        // ros::Duration(5.0).sleep();
        if (push_start)
        {
            // std::cout << "button_floor: " << goal_button << std::endl;
            // buttonType();
            if (setButtonType())
            {
                ros::Duration(5.0).sleep();
                setInitPose();
            }
            else
            {
                ros::Duration(5.0).sleep();
            }
        }
    }
}

}
