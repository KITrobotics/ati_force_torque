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
#include <geometry_msgs/Vector3Stamped.h>

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
    void updateFTData(const ros::TimerEvent& event);
    void visualizeData(double x, double y, double z);

    // create a handle for this node, initialize node
    ros::NodeHandle nh_;

private:
    // CAN parameters
    int canType;
    std::string canPath;
    int canBaudrate;
    int ftsBaseID;
    double nodePubFreq;

    std::string frame_id;
    std::string transform_frame_id;

    // declaration of topics to publish
    ros::Publisher topicPub_transData_;
    ros::Publisher topicPub_ForceData_;
    ros::Publisher topicPub_ForceDataTrans_;
    ros::Publisher topicPub_Marker_;

    // service servers
    ros::ServiceServer srvServer_Init_;
    ros::ServiceServer srvServer_Calibrate_;

    tf2_ros::Buffer *p_tfBuffer;
    tf2_ros::TransformListener* p_tfListener;
    tf2::Transform transform_ee_base;
    geometry_msgs::TransformStamped transform_ee_base_stamped;

    ros::Timer ftUpdateTimer;
    
    bool m_isInitialized;
    ForceTorqueCtrl* p_Ftc;
    std::vector<double> F_avg;
};

ForceTorqueNode::ForceTorqueNode()
{

	m_isInitialized = false;

	topicPub_transData_ = nh_.advertise<geometry_msgs::TransformStamped>("trans_values", 100);

	topicPub_ForceData_ = nh_.advertise<geometry_msgs::WrenchStamped>("force_values", 100);
	topicPub_ForceDataTrans_ = nh_.advertise<geometry_msgs::WrenchStamped>("force_values_transformed", 100);
	topicPub_Marker_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	srvServer_Init_ = nh_.advertiseService("Init", &ForceTorqueNode::srvCallback_Init, this);
	srvServer_Calibrate_ = nh_.advertiseService("Calibrate", &ForceTorqueNode::srvCallback_Calibrate, this);

	// Read data from parameter server
	nh_.param<int>("CAN/type", canType, -1);
	nh_.param<std::string>("CAN/path", canPath, "");
	nh_.param<int>("CAN/baudrate", canBaudrate, -1);
	nh_.param<int>("FTS/base_identifier", ftsBaseID, -1);
	nh_.param<double>("node/ft_pub_freq", nodePubFreq, 100);
	

	ros::NodeHandle nh("~");
	nh.getParam("frame", frame_id);
	nh.getParam("transform_frame", transform_frame_id);

	p_tfBuffer = new tf2_ros::Buffer();
	p_tfListener = new tf2_ros::TransformListener(*p_tfBuffer);
	
	ftUpdateTimer = nh_.createTimer(ros::Rate(nodePubFreq), &ForceTorqueNode::updateFTData, this, false, false);

	p_Ftc = new ForceTorqueCtrl(canType, canPath, canBaudrate, ftsBaseID);
}

bool ForceTorqueNode::srvCallback_Init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res )
{
    if(!m_isInitialized)
    {
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
	    res.error_message.data = "All good, you are nice person! :)";
	    
	    // start timer for reading FT-data
	    ftUpdateTimer.start();
	}
	else {
	    m_isInitialized = false;
	    res.success.data = false;
	    res.error_message.data = "Don't know! But it's bad! :/";
	}
    }
    return true;
}

bool ForceTorqueNode::srvCallback_Calibrate(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res )
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
	
	for(int i = 0; i < measurements; i++) {
	    
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
	
	for(int i = 0; i < 6; i++) {
	    F_avg[i] /= measurements;
	}
	
	res.success.data = true;
	res.error_message.data = "Calibration successfull! :)";
    }
    else {
	res.success.data = false;
	res.error_message.data = "FTS not initialised! :/";
    }
    
    return true;
}

void ForceTorqueNode::updateFTData(const ros::TimerEvent& event)
{
    ros::Time start = ros::Time::now();
    
    double Fx, Fy, Fz, Tx, Ty, Tz = 0;

    p_Ftc->ReadSGData(Fx, Fy, Fz, Tx, Ty, Tz);

    geometry_msgs::WrenchStamped msg, msg_transformed;
    
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
    transform_ee_base_stamped = p_tfBuffer->lookupTransform(transform_frame_id, frame_id, ros::Time(0));
    }
    catch (tf2::TransformException ex ){
    ROS_ERROR("%s",ex.what());
    }

    topicPub_transData_.publish(transform_ee_base_stamped);
    
    geometry_msgs::Vector3Stamped temp_vector_in, temp_vector_out;      
    
    temp_vector_in.header = msg.header;
    temp_vector_in.vector = msg.wrench.force;      
    tf2::doTransform(temp_vector_in, temp_vector_out, transform_ee_base_stamped);      
    msg_transformed.header = temp_vector_out.header;
    msg_transformed.wrench.force = temp_vector_out.vector;
    
    temp_vector_in.vector = msg.wrench.torque;      
    tf2::doTransform(temp_vector_in, temp_vector_out, transform_ee_base_stamped);
    msg_transformed.wrench.torque = temp_vector_out.vector;
    
    topicPub_ForceDataTrans_.publish(msg_transformed);
    
//     ROS_INFO("Duration time of calcuation: %f'", (ros::Time::now() - start).toSec());
//     ROS_INFO("Time between calls: %f", (event.current_real - event.last_real).toSec());
//     ROS_INFO("Error: %f", (event.current_expected - event.current_real).toSec());
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

  ros::init(argc, argv, "forcetorque_node");
  ForceTorqueNode ftn;

  ROS_INFO("ForceTorque Sensor Node running.");

  ros::spin();
  
  return 0;
}

