/****************************************************************
 *
 * Copyright 2016 Intelligent Industrial Robotics (IIROB) Group,
 * Institute for Anthropomatics and Robotics (IAR) -
 * Intelligent Process Control and Robotics (IPR),
 * Karlsruhe Institute of Technology
 *
 * Maintainers: Andreea Tulbure, email: andreea.tulbure@student.kit.edu
 *
 * Date of update: 2016-2017
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
#include <stdint.h>
typedef unsigned char uint8_t;
#include <inttypes.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <cob_forcetorque/ForceTorqueCtrl.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ati_force_torque/PublishConfigurationParameters.h>
#include <ati_force_torque/NodeConfigurationParameters.h>

#include <math.h>
#include <iostream>

#define PI 3.14159265

class ForceTorqueSensorSim
{
public:
  ForceTorqueSensorSim(ros::NodeHandle &nh);
  void init_sensor();

protected:

  std::string transform_frame_;
  std::string sensor_frame_;
  ati_force_torque::NodeConfigurationParameters node_params_;
  ati_force_torque::PublishConfigurationParameters pub_params_;
  void pullFTData(const ros::TimerEvent &event);
  void filterFTData();
  void subscribeData(const geometry_msgs::Twist::ConstPtr& msg);
  // Arrays for dumping FT-Data
  geometry_msgs::WrenchStamped threshold_filtered_force, transformed_data, joystick_data;
  virtual void updateFTData(const ros::TimerEvent &event)  = 0;


private:
  bool transform_wrench(std::string goal_frame, std::string source_frame, geometry_msgs::Wrench wrench, geometry_msgs::Wrench *transformed);
    ros::Subscriber force_input_subscriber;
  uint _num_transform_errors;
  tf2_ros::Buffer *p_tfBuffer;
  tf2_ros::TransformListener *p_tfListener;
  ros::NodeHandle nh_;
  ros::Publisher  transformed_data_pub_,sensor_data_pub_;
  ros::Timer ftUpdateTimer_, ftPullTimer_;
  bool is_pub_transformed_data_ =false;
  bool is_pub_sensor_data_=false;

};

