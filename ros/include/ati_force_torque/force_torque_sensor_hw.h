/****************************************************************
 *
 * Copyright 2016 Intelligent Industrial Robotics (IIROB) Group,
 * Institute for Anthropomatics and Robotics (IAR) -
 * Intelligent Process Control and Robotics (IPR),
 * Karlsruhe Institute of Technology
 *
 * Maintainers: Denis Štogl, email: denis.stogl@kit.edu
 *               Andreea Tulbure
 *
 * Date of update: 2017
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * Author: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 * Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * Date of creation: June 2010
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
#include <ati_force_torque/force_torque_sensor_handle.h>
#include <hardware_interface/sensor_hw.h>

namespace ati_force_torque
{

class ForceTorqueSensorHW : public hardware_interface::SensorHW
{
public:
  ForceTorqueSensorHW();
  virtual ~ForceTorqueSensorHW(){};
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  virtual void read(const ros::Time& time, const ros::Duration& period);
  
  virtual bool stop();
  virtual bool recover();

protected:  
  hardware_interface::ForceTorqueSensorInterface fts_interface_;
  std::string fts_name, fts_transform_frame;
  ForceTorqueSensorHandle* ftsh_;
};

ForceTorqueSensorHW::ForceTorqueSensorHW()
{
}

bool ForceTorqueSensorHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
  // Populate hardware interfaces
  robot_hw_nh.param<std::string>("FTS/fts_name", fts_name, "ATI_45_Mini");
  robot_hw_nh.param<std::string>("Node/transform_frame", fts_transform_frame, "fts_transform_frame");
  
  //if not using handle instatiate directly ForceTorqueSensor
  ftsh_ = new ForceTorqueSensorHandle(robot_hw_nh, fts_name, fts_transform_frame);
  fts_interface_.registerHandle(*ftsh_);
  
  registerInterface(&fts_interface_);

  return true;
}


void ForceTorqueSensorHW::read(const ros::Time& time, const ros::Duration& period)
{
  //example for reading
  double* force; 
  force = ftsh_->getForce();  
  double* torque;
  torque = ftsh_->getTorque();  
}

 bool ForceTorqueSensorHW::stop() 
 {
     ROS_INFO("to be implemented"); 
     return true;
 }
 
 bool ForceTorqueSensorHW::recover() 
 {
     ROS_INFO("to be implemented"); 
     return true;
 };
}


