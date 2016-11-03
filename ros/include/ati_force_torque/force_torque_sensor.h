/****************************************************************
 *
 * Copyright 2016 Intelligent Industrial Robotics (IIROB) Group,
 * Institute for Anthropomatics and Robotics (IAR) -
 * Intelligent Process Control and Robotics (IPR),
 * Karlsruhe Institute of Technology
 *
 * Maintainers: Denis Štogl, email: denis.stogl@kit.edu
 *              Florian Heller
 *              Vanessa Streuer
 *
 * Date of update: 2014-2016
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
#include <stdint.h>
typedef unsigned char uint8_t;
#include <inttypes.h>
#include <iostream>
#include <ros/ros.h>
#include <cob_forcetorque/ForceTorqueCtrl.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/Trigger.h>
#include <ati_force_torque/DiagnosticVoltages.h>


#include <iirob_filters/gravity_compensation.h>
#include <iirob_filters/low_pass_filter.h>
#include <iirob_filters/median_filter.h>
#include <iirob_filters/threshold_filter.h>

#include <math.h>
#include <iostream>
#define PI 3.14159265

class ForceTorqueSensor
{
public:
  ForceTorqueSensor(ros::NodeHandle &nh);


  void init_sensor(std::string &msg, bool &success);
  bool srvCallback_Init(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool srvCallback_Calibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool calibrate();
  bool srvCallback_DetermineCoordinateSystem(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool srvReadDiagnosticVoltages(ati_force_torque::DiagnosticVoltages::Request &req,
                                 ati_force_torque::DiagnosticVoltages::Response &res);
  bool srvCallback_recalibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

protected:

  std::string transform_frame_;
  std::string sensor_frame_;

  void pullFTData(const ros::TimerEvent &event);
  void filterFTData();

  // Arrays for dumping FT-Data
  geometry_msgs::WrenchStamped gravity_compensated_force, median_filtered_wrench, threshold_filtered_force, transformed_data, sensor_data, low_pass_filtered_data;

  double force_buffer_[3];
  double torque_buffer_[3];
  double force_buffer_transformed_[3];
  double torque_buffer_transformed_[3];

private:
  virtual void updateFTData(const ros::TimerEvent &event)  = 0;


  //FT Data
  ForceTorqueCtrl *p_Ftc;
  std::vector<double> F_avg;
  geometry_msgs::TransformStamped transform_ee_base_stamped;
  tf2_ros::Buffer *p_tfBuffer;
  ros::Publisher gravity_compensated_pub_, threshold_filtered_pub_, transformed_data_pub_, sensor_data_pub_, low_pass_pub_;
  bool is_pub_gravity_compensated_, is_pub_threshold_filtered_, is_pub_transformed_data_, is_pub_sensor_data_, is_pub_low_pass_;


  // CAN parameters
  int canType;
  std::string canPath;
  int canBaudrate;
  int ftsBaseID;
  double nodePubFreq;
  int calibrationNMeasurements;
  int calibrationTBetween;
  int coordinateSystemNMeasurements;
  int coordinateSystemTBetween;
  int coordinateSystemPushDirection;

  // service servers
  ros::ServiceServer srvServer_Init_;
  ros::ServiceServer srvServer_Calibrate_;
  ros::ServiceServer srvServer_DetermineCoordianteSystem_;
  ros::ServiceServer srvServer_Temp_;
  ros::ServiceServer srvServer_ReCalibrate;

  ros::Timer ftUpdateTimer_, ftPullTimer_;

  tf2_ros::TransformListener *p_tfListener;
  tf2::Transform transform_ee_base;

  bool m_isInitialized;
  bool m_isCalibrated;

  // Variables for Static offset
  bool m_staticCalibration;
  geometry_msgs::Wrench m_calibOffset;

  ThresholdFilter threshold_filter_;

  LowPassFilter lp_filter_force_x_;
  LowPassFilter lp_filter_force_y_;
  LowPassFilter lp_filter_force_z_;
  LowPassFilter lp_filter_torque_x_;
  LowPassFilter lp_filter_torque_y_;
  LowPassFilter lp_filter_torque_z_;
  MedianFilter median_filter_force_x_;
  MedianFilter median_filter_force_y_;
  MedianFilter median_filter_force_z_;
  MedianFilter median_filter_torque_x_;
  MedianFilter median_filter_torque_y_;
  MedianFilter median_filter_torque_z_;

  GravityCompensator gravity_compensator_;

  ros::NodeHandle nh_;
};
