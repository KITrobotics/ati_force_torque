/****************************************************************
 *
 * Copyright 2016 Intelligent Industrial Robotics (IIROB) Group,
 * Institute for Anthropomatics and Robotics (IAR) -
 * Intelligent Process Control and Robotics (IPR),
 * Karlsruhe Institute of Technology
 *
 * Maintainers: Denis Štogl, email: denis.stogl@kit.edu
 *                     Florian Heller
 *                     Vanessa Streuer
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
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/Trigger.h>
#include <ati_force_torque/CalculateAverageMasurement.h>
#include <ati_force_torque/CalculateSensorOffset.h>
#include <ati_force_torque/DiagnosticVoltages.h>


#include <iirob_filters/gravity_compensation.h>
#include <iirob_filters/GravityCompensationParameters.h>
#include <iirob_filters/low_pass_filter.h>
#include <iirob_filters/moving_mean_filter.h>
#include <iirob_filters/threshold_filter.h>

#include <math.h>
#include <iostream>

#include <dynamic_reconfigure/server.h>
#include <ati_force_torque/CoordinateSystemCalibrationParameters.h>
#include <ati_force_torque/CanConfigurationParameters.h>
#include <ati_force_torque/FTSConfigurationParameters.h>
#include <ati_force_torque/PublishConfigurationParameters.h>
#include <ati_force_torque/PublishConfigurationConfig.h>
#include <ati_force_torque/NodeConfigurationParameters.h>
#include <ati_force_torque/CalibrationParameters.h>
#include <ati_force_torque/CalibrationConfig.h>

#define PI 3.14159265

class ForceTorqueSensor
{
public:
  ForceTorqueSensor(ros::NodeHandle &nh);


  void init_sensor(std::string &msg, bool &success);
  bool srvCallback_Init(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool srvCallback_CalculateOffset(ati_force_torque::CalculateSensorOffset::Request &req, ati_force_torque::CalculateSensorOffset::Response &res);
  bool srvCallback_CalculateAverageMasurement(ati_force_torque::CalculateAverageMasurement::Request &req, ati_force_torque::CalculateAverageMasurement::Response &res);
  bool calibrate(bool apply_after_calculation,  geometry_msgs::Wrench *new_offset);
  bool srvCallback_DetermineCoordinateSystem(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool srvReadDiagnosticVoltages(ati_force_torque::DiagnosticVoltages::Request &req,
                                 ati_force_torque::DiagnosticVoltages::Response &res);
  bool srvCallback_recalibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

protected:
  ati_force_torque::CoordinateSystemCalibrationParameters CS_params_;
  ati_force_torque::CanConfigurationParameters can_params_;
  ati_force_torque::FTSConfigurationParameters FTS_params_;
  ati_force_torque::PublishConfigurationParameters pub_params_;
  ati_force_torque::NodeConfigurationParameters node_params_;
  ati_force_torque::CalibrationParameters calibration_params_;
  iirob_filters::GravityCompensationParameters gravity_params_;

  std::string transform_frame_;
  std::string sensor_frame_;

  void pullFTData(const ros::TimerEvent &event);
  void filterFTData();

  // Arrays for dumping FT-Data
  geometry_msgs::WrenchStamped gravity_compensated_force, moving_mean_filtered_wrench, threshold_filtered_force, transformed_data, sensor_data, low_pass_filtered_data;

  double force_buffer_[3];
  double torque_buffer_[3];
  double force_buffer_transformed_[3];
  double torque_buffer_transformed_[3];
  
  ros::NodeHandle nh_;

private:
  virtual void updateFTData(const ros::TimerEvent &event)  = 0;
  geometry_msgs::Wrench makeAverageMeasurement(uint number_of_measurements, double time_between_meas, std::string frame_id="");
  bool transform_wrench(std::string goal_frame, std::string source_frame, geometry_msgs::Wrench wrench, geometry_msgs::Wrench *transformed);

  //FT Data
  ForceTorqueCtrl *p_Ftc;
  geometry_msgs::Wrench offset_;
  geometry_msgs::TransformStamped transform_ee_base_stamped;
  tf2_ros::Buffer *p_tfBuffer;
  ros::Publisher gravity_compensated_pub_, threshold_filtered_pub_, transformed_data_pub_, sensor_data_pub_, low_pass_pub_, moving_mean_pub_;
  bool is_pub_gravity_compensated_ = false;
  bool is_pub_threshold_filtered_ = false;
  bool is_pub_transformed_data_ = false;
  bool is_pub_sensor_data_ = false;
  bool is_pub_low_pass_ = false;
  bool is_pub_moving_mean_ = false;
  
  uint _num_transform_errors;

  // CAN parameters
  int canType;
  std::string canPath;
  int canBaudrate;
  int ftsBaseID;
  double nodePubFreq;
  uint calibrationNMeasurements;
  double calibrationTBetween;
  int coordinateSystemNMeasurements;
  int coordinateSystemTBetween;
  int coordinateSystemPushDirection;

  // service servers
  ros::ServiceServer srvServer_Init_;
  ros::ServiceServer srvServer_CalculateAverageMasurement_;
  ros::ServiceServer srvServer_CalculateOffset_;
  ros::ServiceServer srvServer_DetermineCoordianteSystem_;
  ros::ServiceServer srvServer_Temp_;
  ros::ServiceServer srvServer_ReCalibrate;

  ros::Timer ftUpdateTimer_, ftPullTimer_;

  tf2_ros::TransformListener *p_tfListener;
  tf2::Transform transform_ee_base;

  bool m_isInitialized;
  bool m_isCalibrated;
  bool apply_offset;

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
  MovingMeanFilter moving_mean_filter_force_x_;
  MovingMeanFilter moving_mean_filter_force_y_;
  MovingMeanFilter moving_mean_filter_force_z_;
  MovingMeanFilter moving_mean_filter_torque_x_;
  MovingMeanFilter moving_mean_filter_torque_y_;
  MovingMeanFilter moving_mean_filter_torque_z_;

  GravityCompensator gravity_compensator_; 

  bool useLowPassFilterForceX=false;  
  bool useLowPassFilterForceY=false;
  bool useLowPassFilterForceZ=false;
  bool useLowPassFilterTorqueX=false;
  bool useLowPassFilterTorqueY=false;
  bool useLowPassFilterTorqueZ=false;
  bool useMovinvingMeanForceX= false;
  bool useMovinvingMeanForceY= false;
  bool useMovinvingMeanForceZ= false;
  bool useMovinvingMeanTorqueX= false;
  bool useMovinvingMeanTorqueY= false;
  bool useMovinvingMeanTorqueZ= false;
  bool useGravityCompensator=false;
  bool useThresholdFilter=false;
  
  dynamic_reconfigure::Server<ati_force_torque::CalibrationConfig> reconfigCalibrationSrv_; // Dynamic reconfiguration service
  dynamic_reconfigure::Server<ati_force_torque::PublishConfigurationConfig> reconfigPublishSrv_; // Dynamic reconfiguration service

  void reconfigureCalibrationRequest(ati_force_torque::CalibrationConfig& config, uint32_t level);
  void reconfigurePublishRequest(ati_force_torque::PublishConfigurationConfig& config, uint32_t level);
};

