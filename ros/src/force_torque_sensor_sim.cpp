/****************************************************************
 *
 * Copyright 2016 Intelligent Industrial Robotics (IIROB) Group,
 * Institute for Anthropomatics and Robotics (IAR) -
 * Intelligent Process Control and Robotics (IPR),
 * Karlsruhe Institute of Technology
 *
 * Maintainers: Denis Štogl, email: denis.stogl@kit.edu
 *                     Andreea Tulbure
 *
 * Date of update: 2014-2017
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

#include <ati_force_torque/force_torque_sensor_sim.h>

ForceTorqueSensorSim::ForceTorqueSensorSim(ros::NodeHandle& nh) : nh_(nh), pub_params_{nh}, node_params_{nh}
{
    std::cout<<"ForceTorqueSensorSim"<<std::endl;

    pub_params_.setNamespace(nh_.getNamespace()+"/Publish");
    node_params_.setNamespace(nh_.getNamespace()+"/Node");
    pub_params_.fromParamServer();
    node_params_.fromParamServer();
     
     //Wrench Publisher  
    is_pub_sensor_data_=pub_params_.sensor_data;
    if(is_pub_sensor_data_){
        sensor_data_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("sensor_data", 1);
    }
    is_pub_transformed_data_  = pub_params_.transformed_data;
    if(is_pub_transformed_data_){
        transformed_data_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("transformed_data", 1);
    }
    ftUpdateTimer_ = nh.createTimer(ros::Rate(node_params_.ft_pull_freq), &ForceTorqueSensorSim::updateFTData, this, false, false);
    
    p_tfBuffer = new tf2_ros::Buffer();
    p_tfListener = new tf2_ros::TransformListener(*p_tfBuffer, true);
    init_sensor();
    ftUpdateTimer_.start();
}
void ForceTorqueSensorSim::init_sensor() {
    force_input_subscriber = nh_.subscribe("twist_controller/command", 1, &ForceTorqueSensorSim::pullFTData, this);
}
void ForceTorqueSensorSim::pullFTData(const geometry_msgs::Twist::ConstPtr &msg)
{
    int status = 0;
    std::string transform_frame_ = "";
    joystick_data.wrench.force.x = msg->linear.x;
    joystick_data.wrench.force.y = msg->linear.y;
    joystick_data.wrench.force.z = 0;
    joystick_data.wrench.torque.z = msg->angular.z;
    joystick_data.wrench.torque.x = 0;
    joystick_data.wrench.torque.y = 0;
    transformed_data.header.stamp = ros::Time::now();
    transformed_data.header.frame_id = transform_frame_;
    transform_wrench(transform_frame_, sensor_frame_, joystick_data.wrench, &transformed_data.wrench);
    threshold_filtered_force = transformed_data;
      if(is_pub_sensor_data_)
	sensor_data_pub_.publish(joystick_data);
     if(is_pub_transformed_data_)
	transformed_data_pub_.publish(threshold_filtered_force);
}
bool ForceTorqueSensorSim::transform_wrench(std::string goal_frame, std::string source_frame, geometry_msgs::Wrench wrench, geometry_msgs::Wrench *transformed)
{
  geometry_msgs::TransformStamped transform;
  geometry_msgs::Vector3Stamped temp_vector_in, temp_vector_out;
  
  try
    {
        transform = p_tfBuffer->lookupTransform(goal_frame, source_frame, ros::Time(0));
	_num_transform_errors = 0;
    }
    catch (tf2::TransformException ex)
    {
      if (_num_transform_errors%100 == 0){
	ROS_ERROR("%s", ex.what());
      }
      _num_transform_errors++;
      return false;
    }

    temp_vector_in.vector = wrench.force;
    tf2::doTransform(temp_vector_in, temp_vector_out, transform);
    transformed->force = temp_vector_out.vector;

    temp_vector_in.vector = wrench.torque;
    tf2::doTransform(temp_vector_in, temp_vector_out, transform);
    transformed->torque = temp_vector_out.vector;
    
    return true;  
}
