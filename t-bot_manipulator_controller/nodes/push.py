#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

import rospy, roslaunch
# import numpy as np
import subprocess
import os
import sys
from enum import Enum
from std_msgs.msg import UInt8, Float32MultiArray
from tf.transformations import *
import tf
# from PySide import QtCore, QtGui, QtOpenGL
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import Int32, String
from math import pow, atan2, sqrt
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import MarkerArray


# Manipulator 
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.msg import KinematicsPose
from open_manipulator_msgs.msg import OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import SetKinematicsPose
from open_manipulator_msgs.srv import GetJointPosition
from open_manipulator_msgs.srv import GetKinematicsPose
from open_manipulator_msgs.srv import SetActuatorState
 
class PickAndPlace():
    def __init__(self):     
        self.push_start = False  

        self.CurrentMode = Enum('CurrentMode', 
                                   'idle \
                                    init \
                                    waitObject \
                                    move_to_pick \
                                    close_object \
                                    move_to_place' )   

        self.listener = tf.TransformListener()
        self.jointStates = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.kinematicsStates = [0.0, 0.0, 0.0]
        self.open_manipulator_moving_state = "STOPPED"
        self.current_mode = self.CurrentMode.init.value
        self.pickObjectPose = PoseStamped()
        self.pickTargetPose = PoseStamped()    
        self.placeObjectPose = PoseStamped()
        self.placeTargetPose = PoseStamped()         
        self.is_triggered = False
        self.currentToolPose = Pose()
        self.use_platform = rospy.get_param("~use_platform","true")
        self.button_floor = ""
        
        self.set_joint_position = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)
        self.set_kinematics_position = rospy.ServiceProxy('goal_task_space_path_position_only', SetKinematicsPose)
        self.set_joint_position_from_present = rospy.ServiceProxy('goal_joint_space_path_from_present', SetJointPosition)
        self.set_actuator_state = rospy.ServiceProxy('set_actuator_state', SetActuatorState)
        self.set_gripper_control = rospy.ServiceProxy('goal_tool_control', SetJointPosition)

        self.open_manipulator_joint_states_sub_ = rospy.Subscriber('joint_states', JointState, self.jointStatesCallback)
        self.open_manipulator_kinematics_pose_sub_ = rospy.Subscriber('gripper/kinematics_pose', KinematicsPose, self.kinematicsPoseCallback)
        self.open_manipulator_states_sub = rospy.Subscriber('states', OpenManipulatorState, self.statesCallback)
        # self.maker_point = rospy.Subscriber('ar_marker_pose', AlvarMarkers, self.markerCallback)
        # self.marker_point = rospy.Subscriber('/button_point', Point, self.markerCallback)
        self.marker_point = rospy.Subscriber('/button_tracker_3d/markers', MarkerArray, self.markerCallback)
        self.floor = rospy.Subscriber('/floor', String, self.floorCallback)
        # self.object_sub = rospy.Subscriber('objects', Float32t-bot_manipulator_controllerMultiArray, self.objectCallback)
        
        rospy.sleep(1)
        # actuator enable 
        self.actuatorTorque(True)
        # self.setInitPose()

        loop_rate = rospy.Rate(10) # 10hz
        
        while not rospy.is_shutdown() :
            if self.is_triggered == True:
                self.setInitPose()
                # print("============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ...")
                # raw_input()
                # self.fnControlNode()

                if self.push_start == True:
                    print(self.push_start)
                    self.moveToObject()
                    rospy.sleep(2.0)
                    pass
                else:
                    self.setInitPose()
                    pass
            loop_rate.sleep()

    def kinematicsPoseCallback(self, msg):
        self.currentToolPose = msg.pose
        rospy.logwarn(' currentToolPose x,y,z %.2f , %.2f, %.2f  ', \
                        self.currentToolPose.position.x, self.currentToolPose.position.y, self.currentToolPose.position.z )

    def actuatorTorque(self, enable):
        rospy.logwarn("actuatorTorque")
        joint_name = ['joint1','joint2','joint3','joint4','gripper']
        try:                  
            resp = self.set_actuator_state(enable)
            rospy.sleep(1)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)  
            return False
        if not resp :
            rospy.loginfo("set_actuator enable fail")        
        return resp

    def setInitPose(self):
        rospy.logwarn("setInitPose")
        # init position
        joint_position = JointPosition()
        joint_position.joint_name = ['joint1','joint2','joint3','joint4']  
        joint_position.position =  [0.0, -1.05, 0.32, 0.70]   
        #joint_position.position =  [0.0, -1.791, 0.507, 1.438]     
        resp = False    
        try:    
            path_time = 2                    
            resp = self.set_joint_position("",joint_position, path_time)
            rospy.sleep(path_time)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e) 
        if not resp :
            return False

    def markerCallback(self, msg):
        for tag in msg.markers:
            if tag.id == 0:
                self.push_start = True
                self.pickObjectPose.header = tag.header
                self.pickObjectPose.pose = tag.pose
            else:
                self.push_start = False
        
        # self.pickObjectPose.header = msg.header
        # for tag in msg.markers:
        #     # x_total = 0
        #     # y_total = 0
        #     # z_total = 0
        #     if tag.id == 0:
        #         self.pickObjectPose.pose = tag.pose.pose


    def floorCallback(self, msg):
        button_info = str(msg)
        button_floor = int(button_info[25:26])
        # rospy.loginfo(button_floor)


    def setBackwardPose(self):
        rospy.logwarn("setInitPose")
        # init position
        joint_position = JointPosition()
        joint_position.joint_name = ['joint1','joint2','joint3','joint4']  
        #joint_position.position =  [0.0, -1.05, 0.35, 0.70]   
        joint_position.position =  [0.0, -1.791, 0.507, 1.438]     
        resp = False    
        try:    
            path_time = 2                    
            resp = self.set_joint_position("",joint_position, path_time)
            rospy.sleep(path_time)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)  
        if not resp :
            return False               

        # open gripper 
        joint_position = JointPosition()
        joint_position.joint_name = ['gripper']  
        joint_position.position =  [0.01] #-0.01 0.01
        resp = False
        try:    
            path_time = 1                    
            resp = self.set_gripper_control("",joint_position, path_time)
            rospy.sleep(path_time)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)  
        if not resp :
            return False 

        return True         

    def setBackwardPose2(self):
        rospy.logwarn("setInitPose")
        # init position
        joint_position = JointPosition()
        joint_position.joint_name = ['joint1','joint2','joint3','joint4']  
        #joint_position.position =  [0.0, -1.05, 0.35, 0.70]   
        joint_position.position =  [0.0, -1.791, 0.507, 1.438]     
        resp = False    
        try:    
            path_time = 2                    
            resp = self.set_joint_position("",joint_position, path_time)
            rospy.sleep(path_time)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)  
        if not resp :
            return False               

        return True

    def moveToObject(self):
        rospy.logwarn("move to object")
        resp = False
        end_effector_name = "gripper" 
        kinematics_pose = KinematicsPose()
        planning_group = "arm"        
        # kinematics_pose.pose = self.pickTargetPose.pose
        kinematics_pose.pose = self.pickObjectPose.pose

        rospy.loginfo(self.pickObjectPose.pose)

        #--------------------------------------------------------------------------#
        # 버튼인식 이용 시 출력되는 좌표의 x, z좌표에 +0.05를 해야 정확한 위치로 이동
        #--------------------------------------------------------------------------#

        kinematics_pose.pose.position = self.forwardObjectPosition( kinematics_pose.pose.position, 0.05 )
        kinematics_pose.pose.position.y -= 0.025
        kinematics_pose.pose.position.z += 0.025

        moveDistance = math.sqrt((kinematics_pose.pose.position.x - self.currentToolPose.position.x)**2 
                               + (kinematics_pose.pose.position.y - self.currentToolPose.position.y)**2       
                               + (kinematics_pose.pose.position.z - self.currentToolPose.position.z)**2 )

        #distance 0.3 m -> 3 sec operate time 
        #distance 0.1 m -> 1 sec operate time        

        operating_time = moveDistance * 10
        operating_limit_time = operating_time

        if operating_time < 1 :
            operating_limit_time = 1
        elif operating_time > 3 :
            operating_limit_time = 3        

        rospy.logwarn("go xyz %.2f,%.2f,%.2f , moveDistance %.2f, operate time %.2f ( %.2f )" ,\
                       kinematics_pose.pose.position.x, kinematics_pose.pose.position.y, kinematics_pose.pose.position.z, \
                       moveDistance, operating_time , operating_limit_time)       

        try:
            resp = self.set_kinematics_position(planning_group, end_effector_name, kinematics_pose, operating_time)
            print('kinemetics resp1 {} time '.format(resp.is_planned, operating_time))
            rospy.sleep(operating_time)
        except rospy.ServiceException:
            print("Service call failed: %s"%e)
            return False

        return resp

    def forwardObjectPosition( self, objectPosition, forward_distance ):
        resultPoint = Point()
        if(abs(objectPosition.x) < 0.001) :
            objectPosition.x = 0.001
        radian = math.atan(objectPosition.y/objectPosition.x)
        degree = math.degrees(radian)
        dist = forward_distance
        distX = math.cos(radian)*dist
        distY = math.sin(radian)*dist
        resultPoint.x = objectPosition.x + distX 
        resultPoint.y = objectPosition.y + distY
        resultPoint.z = objectPosition.z 
        rospy.loginfo("%.2f m forward,so objectposition change xyz(%.2f ,%.2f, %.2f) -> xyz(%.2f ,%.2f, %.2f)",
                       forward_distance, objectPosition.x, objectPosition.y, objectPosition.z , 
                       resultPoint.x, resultPoint.y, resultPoint.z)
        return resultPoint        

    def kinematicsPoseCallback(self, msg):
        self.kinematicsStates[0] = msg.pose.position.x
        self.kinematicsStates[1] = msg.pose.position.y
        self.kinematicsStates[2] = msg.pose.position.z
        #rospy.logwarn(' kinematicsPoseCallback %.2f , %.2f, %.2f  ', self.kinematicsStates[0], self.kinematicsStates[1], self.kinematicsStates[2] )

    def jointStatesCallback(self, msg):
	    #rospy.logwarn('jointStatesCallback %d ', len(msg.position) )
        self.is_triggered = True
        for i, pose in enumerate(msg.position):
            self.jointStates[i] = pose
            #print 'boundingBoxe {} {} '.format(i, pose)            

    def statesCallback(self, msg):	
        self.open_manipulator_moving_state = msg.open_manipulator_moving_state

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('push_node_controller')
    rospy.loginfo("push_node_controller")
    node = PickAndPlace()
    node.main()