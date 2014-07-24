/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_forcetorque
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 * Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * Date of creation: June 2010
 * ToDo:
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
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
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

#include <cob_srvs/Trigger.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/Marker.h>

#include <math.h>
#include <iostream>
#define PI 3.14159265


class ForceTorqueNode
{
public:

	ForceTorqueNode();

	bool srvCallback_Init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res);
  bool srvCallback_Calibrate(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res);
  void updateFTData();
  void visualizeData(double x, double y, double z);

  // create a handle for this node, initialize node
  ros::NodeHandle nh_;

private:
	// CAN parameters
	int deviceType;
	std::string devicePath;
	int deviceBaudrate;
	int deviceBaseIdentifier;

	std::string frame_id;

  // declaration of topics to publish
  ros::Publisher topicPub_ForceData_;
  ros::Publisher topicPub_ForceDataBase_;
  ros::Publisher topicPub_Marker_;

  // service servers
  ros::ServiceServer srvServer_Init_;
  ros::ServiceServer srvServer_Calibrate_;

  tf2_ros::Buffer *p_tfBuffer;
  tf2_ros::TransformListener* p_tfListener;
  tf2::Transform transform_ee_base;
  geometry_msgs::TransformStamped transform_ee_base_stamped;

  bool m_isInitialized;
  ForceTorqueCtrl* p_Ftc;
  std::vector<double> F_avg;

};

ForceTorqueNode::ForceTorqueNode()
{

  m_isInitialized = false;

  topicPub_ForceData_ = nh_.advertise<geometry_msgs::WrenchStamped>("force_values", 100);
  topicPub_ForceDataBase_ = nh_.advertise<geometry_msgs::WrenchStamped>("force_values_base", 100);
  topicPub_Marker_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
  srvServer_Init_ = nh_.advertiseService("Init", &ForceTorqueNode::srvCallback_Init, this);
  srvServer_Calibrate_ = nh_.advertiseService("Calibrate", &ForceTorqueNode::srvCallback_Calibrate, this);

	// Read data from parameter server
	nh_.param<int>("device/type", deviceType, -1);
	nh_.param<std::string>("device/path", devicePath, "");
	nh_.param<int>("device/baudrate", deviceBaudrate, -1);
	nh_.param<int>("device/base_identifier", deviceBaseIdentifier, -1);

	nh_.param<std::string>("frame", frame_id, "fts_link");

	p_tfBuffer = new tf2_ros::Buffer();
	p_tfListener = new tf2_ros::TransformListener(*p_tfBuffer);

	p_Ftc = new ForceTorqueCtrl(deviceType, devicePath, deviceBaudrate, deviceBaseIdentifier);

}

bool ForceTorqueNode::srvCallback_Init(cob_srvs::Trigger::Request &req,
		      cob_srvs::Trigger::Response &res )
{
  if(!m_isInitialized)
    {
		p_Ftc->SetFXGain(-1674.08485641479, 25.3936432491561, 3936.02718786968, -26695.2539299392, -3463.73728677908, 32320.8777656041);
		p_Ftc->SetFYGain(-4941.11252317989, 32269.5827812235, 1073.82949467087, -15541.8400780814, 3061.89541712948, -18995.9891819409);
		p_Ftc->SetFZGain(39553.9250733854, -501.940034213822, 40905.2545309848, 85.1095865539103, 38879.4015426067, 541.344775537753);
		p_Ftc->SetTXGain(-57.4775857386444, 225.941430274037, -638.238694389357, -116.780649376712, 645.133934885308, -116.310081348745 );
		p_Ftc->SetTYGain(786.70602313107, -4.36504382717595, -422.360387149734, 180.7428885668, -352.389412256677, -232.293941041101);
		p_Ftc->SetTZGain(60.1009854270179, -400.19573754971, 29.142908672741, -392.119024237625, 70.9306507180567, -478.104759057292);

		p_Ftc->SetCalibMatrix();
		// read return init status and check it!
		if (p_Ftc->Init()) {
			ROS_INFO("FTC initialized");

			//set Calibdata to zero
			F_avg.resize(6);
			F_avg[0] = 0.0;
			F_avg[1] = 0.0;
			F_avg[2] = 0.0;
			F_avg[3] = 0.0;
			F_avg[4] = 0.0;
			F_avg[5] = 0.0;


			m_isInitialized = true;
			res.success.data = true;
		}
		else {
			m_isInitialized = false;
			res.success.data = false;
			res.error_message.data = "Don't know! But it's bad! :/";
		}
    }
  return m_isInitialized;
}

bool ForceTorqueNode::srvCallback_Calibrate(cob_srvs::Trigger::Request &req,
		      cob_srvs::Trigger::Response &res )
{
  int measurements = 20;
  if(m_isInitialized)
    {
      F_avg[0] = 0.0;
      F_avg[1] = 0.0;
      F_avg[2] = 0.0;
      F_avg[3] = 0.0;
      F_avg[4] = 0.0;
      F_avg[5] = 0.0;
      for(int i = 0; i < measurements; i++)
	{
	  double Fx, Fy, Fz, Tx, Ty, Tz = 0;
	  p_Ftc->ReadSGData(Fx, Fy, Fz, Tx, Ty, Tz);
	  F_avg[0] += Fx;
	  F_avg[1] += Fy;
	  F_avg[2] += Fz;
	  F_avg[3] += Tx;
	  F_avg[4] += Ty;
	  F_avg[5] += Tz;
	  usleep(10000);
	}
      for(int i = 0; i < 6; i++)
	F_avg[i] /= measurements;
      return true;
    }
  else
    return false;
}

void ForceTorqueNode::updateFTData()
{
  if(m_isInitialized)
    {
      double Fx, Fy, Fz, Tx, Ty, Tz = 0;

      p_Ftc->ReadSGData(Fx, Fy, Fz, Tx, Ty, Tz);

      geometry_msgs::WrenchStamped msg;
			msg.header.frame_id = frame_id;
			msg.header.stamp = ros::Time::now();
			msg.wrench.force.x = Fx-F_avg[0];
			msg.wrench.force.y = Fy-F_avg[1];
			msg.wrench.force.z = Fz-F_avg[2];
			msg.wrench.torque.x = Tx-F_avg[3];
			msg.wrench.torque.y = Ty-F_avg[4];
			msg.wrench.torque.z = Tz-F_avg[5];
      topicPub_ForceData_.publish(msg);

      tf2::Transform fdata_base;
      tf2::Transform fdata;
      fdata.setOrigin(tf2::Vector3(Fx-F_avg[0], Fy-F_avg[1], Fz-F_avg[2]));

      try{
        transform_ee_base_stamped = p_tfBuffer->lookupTransform("base_link", frame_id, ros::Time(0));
      }
      catch (tf2::TransformException ex ){
				ROS_ERROR("%s",ex.what());
      }

      // TODO

//       geometry_msgs::PoseStamped pose;
//
// 			pose.header = transform_ee_base_stamped.header;
// 			pose.pose.position.x = transform_ee_base_stamped.transform.translation.x;
// 			pose.pose.position.y = transform_ee_base_stamped.transform.translation.y;
// 			pose.pose.position.z = transform_ee_base_stamped.transform.translation.z;
//
// 			pose.pose.orientation.x = transform_ee_base_stamped.transform.rotation.x;
// 			pose.pose.orientation.y = transform_ee_base_stamped.transform.rotation.y;
// 			pose.pose.orientation.z = transform_ee_base_stamped.transform.rotation.z;
// 			pose.pose.orientation.w = transform_ee_base_stamped.transform.rotation.w;
//
//       tf2::convert(pose, transform_ee_base);
//
//       fdata_base = transform_ee_base * fdata;
// 			geometry_msgs::WrenchStamped base_msg;
// 			base_msg.header.frame_id = frame_id;
// 			base_msg.header.stamp = ros::Time::now();
// 			base_msg.wrench.force.x = fdata_base.getOrigin().x();
// 			base_msg.wrench.force.y = fdata_base.getOrigin().y();
// 			base_msg.wrench.force.z = fdata_base.getOrigin().z();
// 			base_msg.wrench.torque.x = 0.0;
// 			base_msg.wrench.torque.y = 0.0;
// 			base_msg.wrench.torque.z = 0.0;
//       topicPub_ForceDataBase_.publish(base_msg);
//       visualizeData(fdata_base.getOrigin().x(), fdata_base.getOrigin().y(), fdata_base.getOrigin().z());
    }
}

void ForceTorqueNode::visualizeData(double x, double y, double z)
{
  visualization_msgs::Marker marker;
  uint32_t shape = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "ForceTorqueData";
  marker.id = 0;
  marker.type = shape;
// first delete old markers
  marker.action = visualization_msgs::Marker::DELETE;
  topicPub_Marker_.publish(marker);
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.scale.x = x/100;
  marker.scale.y = y/100;
  marker.scale.z = z/100;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  topicPub_Marker_.publish(marker);
}






int main(int argc, char ** argv)
{

  ros::init(argc, argv, "talker");
  ForceTorqueNode ftn;

  ROS_INFO("ForceTorque Sensor Node running.");

  ros::Rate loop_rate(10);
  while(ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
      ftn.updateFTData();
    }
  return 0;
}

