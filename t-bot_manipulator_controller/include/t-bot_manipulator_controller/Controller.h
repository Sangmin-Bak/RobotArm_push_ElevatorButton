#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <math.h>

// Manipulator
#include "open_manipulator_msgs/JointPosition.h"
#include "open_manipulator_msgs/KinematicsPose.h"
#include "open_manipulator_msgs/OpenManipulatorState.h"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/GetJointPosition.h"
#include "open_manipulator_msgs/GetKinematicsPose.h"
#include "open_manipulator_msgs/SetActuatorState.h"

class Push
{
    private:
    ros::NodeHandle nh;
    tf::TransformListener transform_listener;
    bool push_start = false;
    float jointState[10];
};