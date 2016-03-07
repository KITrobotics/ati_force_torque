/****************************************************************
 *
 * Copyright 2016 Intelligent Industrial Robotics (IIROB) Group,
 * Institute for Anthropomatics and Robotics (IAR) -
 * Intelligent Process Control and Robotics (IPR),
 * Karlsruhe Institute of Technology
 *
 * Maintainer: Denis Štogl, email: denis.stogl@kit.edu
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
#include <ati_mini_45/DiagnosticVoltages.h>

#include <math.h>
#include <iostream>
#define PI 3.14159265

class ForceTorqueNode
{
public:
  ForceTorqueNode();

  bool srvCallback_Init(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool srvCallback_Calibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool calibrate();
  bool srvCallback_DetermineCoordinateSystem(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool srvReadDiagnosticVoltages(ati_mini_45::DiagnosticVoltages::Request &req,
                                 ati_mini_45::DiagnosticVoltages::Response &res);
  bool srvCallback_recalibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void updateFTData(const ros::TimerEvent &event);

  // create a handle for this node, initialize node
  ros::NodeHandle nh_;

private:
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

  std::string frame_id;
  std::string transform_frame_id;

  // declaration of topics to publish
  ros::Publisher topicPub_ForceData_;
  ros::Publisher topicPub_ForceDataTrans_;
  ros::Publisher topicPub_Marker_;

  // service servers
  ros::ServiceServer srvServer_Init_;
  ros::ServiceServer srvServer_Calibrate_;
  ros::ServiceServer srvServer_DetermineCoordianteSystem_;
  ros::ServiceServer srvServer_Temp_;
  ros::ServiceServer srvServer_ReCalibrate;

  tf2_ros::Buffer *p_tfBuffer;
  tf2_ros::TransformListener *p_tfListener;
  tf2::Transform transform_ee_base;
  geometry_msgs::TransformStamped transform_ee_base_stamped;

  ros::Timer ftUpdateTimer;

  bool m_isInitialized;
  bool m_isCalibrated;

  // Variables for Static offset
  bool m_staticCalibration;
  geometry_msgs::Wrench m_calibOffset;

  ForceTorqueCtrl *p_Ftc;
  std::vector<double> F_avg;
};

ForceTorqueNode::ForceTorqueNode()
{
  m_isInitialized = false;
  m_isCalibrated = false;

  topicPub_ForceData_ = nh_.advertise<geometry_msgs::WrenchStamped>("force_values", 1);
  topicPub_ForceDataTrans_ = nh_.advertise<geometry_msgs::WrenchStamped>("force_values_transformed", 1);
  srvServer_Init_ = nh_.advertiseService("Init", &ForceTorqueNode::srvCallback_Init, this);
  srvServer_Calibrate_ = nh_.advertiseService("Calibrate", &ForceTorqueNode::srvCallback_Calibrate, this);
  srvServer_DetermineCoordianteSystem_ =
      nh_.advertiseService("DetermineCoordinateSystem", &ForceTorqueNode::srvCallback_DetermineCoordinateSystem, this);
  srvServer_Temp_ = nh_.advertiseService("GetTemperature", &ForceTorqueNode::srvReadDiagnosticVoltages, this);
  srvServer_ReCalibrate = nh_.advertiseService("Recalibrate", &ForceTorqueNode::srvCallback_recalibrate, this);

  // Read data from parameter server
  nh_.param<int>("CAN/type", canType, -1);
  nh_.param<std::string>("CAN/path", canPath, "");
  nh_.param<int>("CAN/baudrate", canBaudrate, -1);
  nh_.param<int>("FTS/base_identifier", ftsBaseID, -1);
  nh_.param<double>("node/ft_pub_freq", nodePubFreq, 100);
  nh_.param<std::string>("node/frame", frame_id, "fts_link");
  nh_.param<std::string>("node/transform_frame", transform_frame_id, "base_link");
  nh_.param<int>("Calibration/n_measurements", calibrationNMeasurements, 20);
  nh_.param<int>("Calibration/T_between_meas", calibrationTBetween, 10000);
  nh_.param<bool>("Calibration/static", m_staticCalibration, 0);
  nh_.param<double>("Calibration/offset_fx", m_calibOffset.force.x, 0);
  nh_.param<double>("Calibration/offset_fy", m_calibOffset.force.y, 0);
  nh_.param<double>("Calibration/offset_fz", m_calibOffset.force.z, 0);
  nh_.param<double>("Calibration/offset_mx", m_calibOffset.torque.x, 0);
  nh_.param<double>("Calibration/offset_my", m_calibOffset.torque.y, 0);
  nh_.param<double>("Calibration/offset_mz", m_calibOffset.torque.z, 0);
  nh_.param<int>("CoordinateSystemCal/n_measurements", coordinateSystemNMeasurements, 20);
  nh_.param<int>("CoordinateSystemCal/T_between_meas", coordinateSystemTBetween, 10000);
  nh_.param<int>("CoordinateSystemCal/push_direction", coordinateSystemPushDirection, 0);

  p_tfBuffer = new tf2_ros::Buffer();
  p_tfListener = new tf2_ros::TransformListener(*p_tfBuffer, true);

  ftUpdateTimer = nh_.createTimer(ros::Rate(nodePubFreq), &ForceTorqueNode::updateFTData, this, false, false);

  if (canType != -1)
  {
    p_Ftc = new ForceTorqueCtrl(canType, canPath, canBaudrate, ftsBaseID);
  }
  else
  {
    p_Ftc = new ForceTorqueCtrl();
  }
}

bool ForceTorqueNode::srvCallback_Init(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  if (!m_isInitialized)
  {
    // read return init status and check it!
    if (p_Ftc->Init())
    {
      // Calibrate sensor
      F_avg.resize(6);
      if (m_staticCalibration)
      {
        F_avg[0] = m_calibOffset.force.x;
        F_avg[1] = m_calibOffset.force.y;
        F_avg[2] = m_calibOffset.force.z;
        F_avg[3] = m_calibOffset.torque.x;
        F_avg[4] = m_calibOffset.torque.y;
        F_avg[5] = m_calibOffset.torque.z;
      }
      else
      {
        if (calibrate())
        {
          res.success = false;
          res.message = "Calibration failed! :/";
        }
      }

      m_isInitialized = true;
      res.success = true;
      res.message = "All good, you are nice person! :)";

      // start timer for reading FT-data
      ftUpdateTimer.start();
    }
    else
    {
      m_isInitialized = false;
      res.success = false;
      res.message = "FTS could not be initilised! :/";
    }
  }
  return true;
}

bool ForceTorqueNode::srvCallback_Calibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  if (m_isInitialized)
  {
    if (calibrate())
    {
      res.success = true;
      res.message = "Calibration successfull! :)";
    }
    else
    {
      res.success = false;
      res.message = "Calibration failed! :/";
    }
  }
  else
  {
    res.success = false;
    res.message = "FTS not initialised! :/";
  }

  return true;
}

bool ForceTorqueNode::srvCallback_recalibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  if (!m_isInitialized)
  {
    ROS_WARN("FTS-Node is not initialized, please initialize first!");
    res.success = false;
    res.message = "Failed to recalibrate because Node is not initiliazed.";
    return true;
  }
  if (!(nh_.hasParam("force") && nh_.hasParam("CoG/x") && nh_.hasParam("CoG/y") && nh_.hasParam("CoG/z")))
  {
    ROS_ERROR("Cannot use dynamic recalibration without all values for Gravity Compensation, set parameters or use "
              "'Calibrate' service instead.");
    res.success = false;
    res.message = "Failed to recalibrate because of missing Parameters for Gravity Compensation.";
    return true;
  }
  geometry_msgs::Vector3Stamped gravity, gravity_transformed;
  geometry_msgs::Vector3 cog;
  double force_value;
  nh_.getParam("CoG/x", cog.x);
  nh_.getParam("CoG/y", cog.y);
  nh_.getParam("CoG/z", cog.z);
  nh_.getParam("force", force_value);
  gravity.vector.z = -force_value;
  //~ gravity.header.frame_id=transform_frame_id;
  //~ tf2::Transform tf_base_to_kms = p_tfBuffer->lookupTransform(frame_id,transform_frame_id,ros::Time(0));
  tf2::doTransform(gravity, gravity_transformed,
                   p_tfBuffer->lookupTransform(frame_id, transform_frame_id, ros::Time(0)));
  calibrate();
  F_avg[0] -= gravity_transformed.vector.x;
  F_avg[1] -= gravity_transformed.vector.y;
  F_avg[2] -= gravity_transformed.vector.z;
  F_avg[3] -= (gravity_transformed.vector.y * cog.z - gravity_transformed.vector.z * cog.y);
  F_avg[4] -= (gravity_transformed.vector.z * cog.x - gravity_transformed.vector.x * cog.z);
  F_avg[5] -= (gravity_transformed.vector.x * cog.y - gravity_transformed.vector.y * cog.x);
  res.success = true;
  res.message = "Successfully recalibrated FTS!";
  return true;
}

bool ForceTorqueNode::calibrate()
{
  F_avg[0] = 0.0;
  F_avg[1] = 0.0;
  F_avg[2] = 0.0;
  F_avg[3] = 0.0;
  F_avg[4] = 0.0;
  F_avg[5] = 0.0;

  for (int i = 0; i < calibrationNMeasurements; i++)
  {
    int status = 0;
    double Fx, Fy, Fz, Tx, Ty, Tz = 0;
    p_Ftc->ReadSGData(status, Fx, Fy, Fz, Tx, Ty, Tz);
    F_avg[0] += Fx;
    F_avg[1] += Fy;
    F_avg[2] += Fz;
    F_avg[3] += Tx;
    F_avg[4] += Ty;
    F_avg[5] += Tz;
    usleep(calibrationTBetween);
  }

  for (int i = 0; i < 6; i++)
  {
    F_avg[i] /= calibrationNMeasurements;
  }

  ROS_INFO("Calibration Data: Fx: %f; Fy: %f; Fz: %f; Mx: %f; My: %f; Mz: %f", F_avg[0], F_avg[1], F_avg[2], F_avg[3],
           F_avg[4], F_avg[5]);

  m_isCalibrated = true;

  return m_isCalibrated;
}

bool ForceTorqueNode::srvCallback_DetermineCoordinateSystem(std_srvs::Trigger::Request &req,
                                                            std_srvs::Trigger::Response &res)
{
  if (m_isInitialized && m_isCalibrated)
  {
    double angle;

    ROS_INFO("Please push FTS with force larger than 10 N in desired direction of new axis %d",
             coordinateSystemPushDirection);

    for (int i = 0; i < coordinateSystemNMeasurements; i++)
    {
      int status = 0;
      double Fx, Fy, Fz, Tx, Ty, Tz = 0;
      p_Ftc->ReadSGData(status, Fx, Fy, Fz, Tx, Ty, Tz);

      angle += atan2(Fy, Fx);

      usleep(coordinateSystemTBetween);
    }

    angle /= coordinateSystemNMeasurements;

    if (coordinateSystemPushDirection)
    {
      angle -= M_PI / 2;
    }

    ROS_INFO("Please rotate your coordinate system for %f rad (%f deg) around z-axis", angle, angle / M_PI * 180.0);

    res.success = true;
    res.message = "CoordianteSystem  successfull! :)";
  }
  else
  {
    res.success = false;
    res.message = "FTS not initialised or not calibrated! :/";
  }

  return true;
}

bool ForceTorqueNode::srvReadDiagnosticVoltages(ati_mini_45::DiagnosticVoltages::Request &req,
                                                ati_mini_45::DiagnosticVoltages::Response &res)
{
  p_Ftc->ReadDiagnosticADCVoltages(req.index, res.adc_value);

  return true;
}

void ForceTorqueNode::updateFTData(const ros::TimerEvent &event)
{
  int status = 0;
  double Fx, Fy, Fz, Tx, Ty, Tz = 0;

  p_Ftc->ReadSGData(status, Fx, Fy, Fz, Tx, Ty, Tz);

  geometry_msgs::WrenchStamped msg, msg_transformed;

  msg.header.frame_id = frame_id;
  msg.header.stamp = ros::Time::now();
  msg.wrench.force.x = Fx - F_avg[0];
  msg.wrench.force.y = Fy - F_avg[1];
  msg.wrench.force.z = Fz - F_avg[2];
  msg.wrench.torque.x = Tx - F_avg[3];
  msg.wrench.torque.y = Ty - F_avg[4];
  msg.wrench.torque.z = Tz - F_avg[5];
  topicPub_ForceData_.publish(msg);

  try
  {
    transform_ee_base_stamped = p_tfBuffer->lookupTransform(transform_frame_id, frame_id, ros::Time(0));
  }
  catch (tf2::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  geometry_msgs::Vector3Stamped temp_vector_in, temp_vector_out;

  temp_vector_in.header = msg.header;
  temp_vector_in.vector = msg.wrench.force;
  tf2::doTransform(temp_vector_in, temp_vector_out, transform_ee_base_stamped);
  msg_transformed.header.stamp = msg.header.stamp;
  msg_transformed.header.frame_id = temp_vector_out.header.frame_id;
  msg_transformed.wrench.force = temp_vector_out.vector;

  temp_vector_in.vector = msg.wrench.torque;
  tf2::doTransform(temp_vector_in, temp_vector_out, transform_ee_base_stamped);
  msg_transformed.wrench.torque = temp_vector_out.vector;

  topicPub_ForceDataTrans_.publish(msg_transformed);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "forcetorque_node");
  ForceTorqueNode ftn;

  ROS_INFO("ForceTorque Sensor Node running.");

  ros::spin();

  return 0;
}
