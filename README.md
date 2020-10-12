# Robot Arm push the Elevator Button
This project recognizes elevator buttons as cameras and presses them with robot arms.

## Requirements
1. Ubuntu16.04
2. ROS Kinetic
3. Tensorflow 1.14.0 
4. python2.7
5. 4GB GPU

## Installation
```
Install pycuda
pip install pycuda

Install Realsense D435
https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_perceptions/

Install ocr-rcnn-v2
https://github.com/zhudelong/ocr-rcnn-v2

Install OpenManipulator
https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_setup/#ros-setup


cd ~/catkin_ws/src
git clone https://github.com/sm315/RobotArm_push_ElevatorButton.git
cd ~/catkin_ws && catkin_make

cd ~/catkin_ws/src/RobotArm_push_ElevatorButton/open_manipulator_libs/src
cp open_manipulator.cpp ~/catkin_ws/src/open_manipulator_libs/src/

cd ~/catkin_ws/src/RobotArm_push_ElevatorButton/robotis_manipulator
cp -r include/ ~/catkin_ws/src/robotis_manipulator
cp -r src/ ~/catkin_ws/src/robotis_manipulator

roscd button_trakcer_3d/capy_files
cp recognniton_service.py ~/catkin_ws/src/ocr-rcnn-v2/src/button_recognition/scripts

cd ~/catkin_ws && catkin_make
```

## Run
### button detection and push
```
roslaunch open_manipulator_controller open_manipulator_controller.launch
roslaunch open_manipulator_description open_manipulator_rviz.launch 
rosrun button_recognition recognition_service.py
roslaunch button_tracker_3d button_tracker.launch
roslaunch t-bot_manipulator_controller push.launch
```

## Links referenced
1.  button detection : https://github.com/zhudelong/ocr-rcnn-v2
2.  3D position : https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d
3.  Robot Arm Control : https://github.com/minwoominwoominwoo7/open_manipulator_find_object_2d