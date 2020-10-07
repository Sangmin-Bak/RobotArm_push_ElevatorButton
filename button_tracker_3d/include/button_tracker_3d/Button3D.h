// Copyright 2020 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Author: Francisco Martín fmrico@gmail.com */
/* Author: Fernando González fergonzaramos@yahoo.es */

#ifndef BUTTON_TRACKER_3D_BUTTON3D_H
#define BUTTON_TRACKER_3D_BUTTON3D_H

#include <ros/ros.h>
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <button_recognition_msgs/BoundingBoxes.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>

#include <vector>
#include <string>

namespace button_tracker_3d
{

class Button3D
{
public:
  Button3D();

  virtual void update();

private:
  void initParams();
  void pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void buttonCb(const button_recognition_msgs::BoundingBoxes::ConstPtr& msg);
  void publish_markers(const gb_visual_detection_3d_msgs::BoundingBoxes3d& boxes);
  // void button_brodcaster()

  void calculate_boxes(const sensor_msgs::PointCloud2& cloud_pc2,
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_pcl,
      gb_visual_detection_3d_msgs::BoundingBoxes3d* boxes);

  ros::NodeHandle nh_;
  ros::Subscriber yolo_sub_, pointCloud_sub_;
  ros::Publisher button3d_pub_, markers_pub_;
  tf::TransformListener tfListener_;

  std::vector<button_recognition_msgs::BoundingBox> original_bboxes_;
  sensor_msgs::PointCloud2 point_cloud_;
  ros::Time last_detection_ts_;

  std::string input_bbx_topic_;
  std::string output_bbx3d_topic_;
  std::string pointcloud_topic_;
  std::string working_frame_;
  std::vector<std::string> interested_classes_;
  float mininum_detection_thereshold_, minimum_probability_;
};  

}; // namespace button_tracker_3d

#endif // BUTTON_TRACKER_3D_BUTTON3D_H