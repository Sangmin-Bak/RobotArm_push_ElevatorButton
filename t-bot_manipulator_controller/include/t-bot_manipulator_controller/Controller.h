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

#ifndef T_BOT_MANIPULATOR_CONTROLLER_H
#define T_BOT_MANIPULATOR_CONTROLLER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <vector>

// Manipulator
#include "open_manipulator_msgs/JointPosition.h"
#include "open_manipulator_msgs/KinematicsPose.h"
#include "open_manipulator_msgs/OpenManipulatorState.h"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/GetJointPosition.h"
#include "open_manipulator_msgs/GetKinematicsPose.h"
#include "open_manipulator_msgs/SetActuatorState.h"

namespace push_button
{
class PushButton
{
private:

    ros::ServiceClient set_joint_position;
    ros::ServiceClient set_kinematics_position;
    ros::ServiceClient set_joint_position_from_present;
    ros::ServiceClient set_actuator_state;
    ros::ServiceClient set_gripper_control;

    ros::Subscriber open_manipulator_joint_state_sub;
    ros::Subscriber open_manipulator_kinematics_pose_sub;
    ros::Subscriber open_manipulator_states_sub;
    ros::Subscriber marker_point;

    tf::TransformListener transform_listener;
    geometry_msgs::PoseStamped pushButtonPose;
    geometry_msgs::Pose currentToolPose;
    open_manipulator_msgs::KinematicsPose kinematics_pose_;

    bool push_start;
    bool is_triggered;
    bool is_shutdown;
    std::vector<double> jointState;
    std::vector<double> kinematicsStates;
    std::string open_manipulator_moveing_state;

public:

    PushButton();

    void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr& msg);
    void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void statesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr& msg);
    bool setInitPose();
    bool setBackwardPose();
    bool setBackwardPose2();

    void setJointPositionCallback();

    geometry_msgs::Point forwardButtonPosition(geometry_msgs::Point button_position, double forward_distance);
    bool actuatorTorque(bool enable);
    bool moveToButton();

    void update();

    ~PushButton();
};
}
#endif  // T-BOT_MANIPULATOR_CONTROLLER_H