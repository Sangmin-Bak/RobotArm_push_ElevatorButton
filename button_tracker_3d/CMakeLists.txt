cmake_minimum_required(VERSION 3.0.2)
project(button_tracker_3d)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  button_recognition_msgs
  gb_visual_detection_3d_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  roslint
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES button_tracker_3d
 CATKIN_DEPENDS 
 roscpp 
 std_msgs 
 button_recognition_msgs
 pcl_ros
 pcl_conversions
 DEPENDS PCL
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  # /usr/include/pcl-1.7/
  # /usr/include/eigen3/
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/button_tracker_3d/Button3D.cpp
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(button3d_node src/button3d_node.cpp)

add_dependencies(button3d_node ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(button3d_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${PCL_DEFINITIONS})

roslint_cpp(
  src/button_tracker_3d/Button3D.cpp include/button_tracker_3d/Button3D.h
  src/button3d_node.cpp
)

install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})