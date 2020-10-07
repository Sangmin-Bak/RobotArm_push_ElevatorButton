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

#include <ros/ros.h>

#include "button_tracker_3d/Button3D.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "button_3d");
  button_tracker_3d::Button3D button3d;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    button3d.update();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
