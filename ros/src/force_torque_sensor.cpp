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
#include <ati_force_torque/force_torque_sensor.h>

ForceTorqueSensor::ForceTorqueSensor(ros::NodeHandle& nh) : nh_(nh)
{
    bool isAutoInit = false;
    double nodePullFreq = 0;
    m_isInitialized = false;
    m_isCalibrated = false;

    srvServer_Init_ = nh_.advertiseService("Init", &ForceTorqueSensor::srvCallback_Init, this);
    srvServer_CalculateAverageMasurement_ = nh_.advertiseService("CalculateAverageMasurement", &ForceTorqueSensor::srvCallback_CalculateAverageMasurement, this);
    srvServer_CalculateOffset_ = nh_.advertiseService("CalculateOffsets", &ForceTorqueSensor::srvCallback_CalculateOffset, this);
    srvServer_DetermineCoordianteSystem_ = nh_.advertiseService("DetermineCoordinateSystem", &ForceTorqueSensor::srvCallback_DetermineCoordinateSystem, this);
    srvServer_Temp_ = nh_.advertiseService("GetTemperature", &ForceTorqueSensor::srvReadDiagnosticVoltages, this);
    srvServer_ReCalibrate = nh_.advertiseService("Recalibrate", &ForceTorqueSensor::srvCallback_recalibrate, this);

    // Read data from parameter server
    nh_.param<int>("CAN/type", canType, -1);

    nh_.param<std::string>("CAN/path", canPath, "");
    nh_.param<int>("CAN/baudrate", canBaudrate, -1);
    nh_.param<int>("FTS/base_identifier", ftsBaseID, -1);
    nh_.param<bool>("FTS/auto_init", isAutoInit, false);
    nh_.param<double>("Node/ft_pub_freq", nodePubFreq, 100);
    nh_.param<double>("Node/ft_pull_freq", nodePullFreq, 100);
    nh_.param<std::string>("Node/sensor_frame", sensor_frame_, "fts_reference_link");

    int calibNMeas;
    nh_.param<int>("Calibration/n_measurements", calibNMeas, 20);
    if (calibNMeas <= 0)
    {
        ROS_WARN("Parameter 'Calibration/n_measurements' is %d (<=0) using default: 20", calibNMeas);
        calibrationNMeasurements = 20;
    }
    else {
        calibrationNMeasurements = (uint)calibNMeas;
    }
    nh_.param<double>("Calibration/T_between_meas", calibrationTBetween, 0.01);
    nh_.param<bool>("Calibration/static", m_staticCalibration, 0);
    nh_.param<double>("Calibration/Offset/force/x", m_calibOffset.force.x, 0);
    nh_.param<double>("Calibration/Offset/force/y", m_calibOffset.force.y, 0);
    nh_.param<double>("Calibration/Offset/force/z", m_calibOffset.force.z, 0);
    nh_.param<double>("Calibration/Offset/torque/x", m_calibOffset.torque.x, 0);
    nh_.param<double>("Calibration/Offset/torque/y", m_calibOffset.torque.y, 0);
    nh_.param<double>("Calibration/Offset/torque/z", m_calibOffset.torque.z, 0);
    nh_.param<int>("CoordinateSystemCal/n_measurements", coordinateSystemNMeasurements, 20);
    nh_.param<int>("CoordinateSystemCal/T_between_meas", coordinateSystemTBetween, 10000);
    nh_.param<int>("CoordinateSystemCal/push_direction", coordinateSystemPushDirection, 0);

    p_tfBuffer = new tf2_ros::Buffer();
    p_tfListener = new tf2_ros::TransformListener(*p_tfBuffer, true);

    //Wrench Publisher
    nh_.param<bool>("Publish/gravity_compensated", is_pub_gravity_compensated_, false);
    if(is_pub_gravity_compensated_){
        gravity_compensated_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("gravity_compensated", 1);
    }
    nh_.param<bool>("Publish/low_pass", is_pub_low_pass_, false);
    if(is_pub_low_pass_){
        low_pass_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("low_pass", 1);
    }
    nh_.param<bool>("Publish/moving_mean", is_pub_moving_mean_, false);
    if(is_pub_moving_mean_){
        moving_mean_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("moving_mean", 1);
    }
    nh_.param<bool>("Publish/sensor_data", is_pub_sensor_data_, false);
    if(is_pub_sensor_data_){
        sensor_data_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("sensor_data", 1);
    }
    nh_.param<bool>("Publish/threshold_filtered", is_pub_threshold_filtered_, false);
    if(is_pub_threshold_filtered_){
        threshold_filtered_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("threshold_filtered", 1);
    }
    nh_.param<bool>("Publish/transformed_data", is_pub_transformed_data_, false);
    if(is_pub_transformed_data_){
        transformed_data_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("transformed_data", 1);
    }



    ftUpdateTimer_ = nh.createTimer(ros::Rate(nodePubFreq), &ForceTorqueSensor::updateFTData, this, false, false);
    ftPullTimer_ = nh.createTimer(ros::Rate(nodePullFreq), &ForceTorqueSensor::pullFTData, this, false, false);

    //Lowpass Filter
    lp_filter_force_x_.init(ros::NodeHandle(nh_, "LowPassFilter/Force_x"));
    lp_filter_force_y_.init(ros::NodeHandle(nh_, "LowPassFilter/Force_y"));
    lp_filter_force_z_.init(ros::NodeHandle(nh_, "LowPassFilter/Force_z"));
    lp_filter_torque_x_.init(ros::NodeHandle(nh_, "LowPassFilter/Torque_x"));
    lp_filter_torque_y_.init(ros::NodeHandle(nh_, "LowPassFilter/Torque_y"));
    lp_filter_torque_z_.init(ros::NodeHandle(nh_, "LowPassFilter/Torque_z"));

    //Median Filter
    moving_mean_filter_force_x_.init(ros::NodeHandle(nh_, "MovingMeanFilter/Force_x"));
    moving_mean_filter_force_y_.init(ros::NodeHandle(nh_, "MovingMeanFilter/Force_y"));
    moving_mean_filter_force_z_.init(ros::NodeHandle(nh_, "MovingMeanFilter/Force_z"));
    moving_mean_filter_torque_x_.init(ros::NodeHandle(nh_, "MovingMeanFilter/Torque_x"));
    moving_mean_filter_torque_y_.init(ros::NodeHandle(nh_, "MovingMeanFilter/Torque_y"));
    moving_mean_filter_torque_z_.init(ros::NodeHandle(nh_, "MovingMeanFilter/Torque_z"));

    //Gravity Compenstation
    gravity_compensator_.init(ros::NodeHandle(nh_, "GravityCompensation"));

    //Threshold Filter
    threshold_filter_.init(ros::NodeHandle(nh_, "ThresholdFilter"));

    if (canType != -1)
    {
        p_Ftc = new ForceTorqueCtrl(canType, canPath, canBaudrate, ftsBaseID);
    }
    else
    {
        p_Ftc = new ForceTorqueCtrl();
    }

    if (isAutoInit)
    {
        std::string msg;
        bool success;
        init_sensor(msg, success);
        ROS_INFO("Autoinit: %s", msg.c_str());
    }
}



void ForceTorqueSensor::init_sensor(std::string& msg, bool& success)
{
    if (!m_isInitialized)
    {
        // read return init status and check it!
        if (p_Ftc->Init())
        {
            // Calibrate sensor
            if (m_staticCalibration)
            {
                ROS_INFO("Using static calibration from paramter server with parametes Force: x:%f, y:%f, z:%f; Torque: x: %f, y:%f, z:%f;",
		  m_calibOffset.force.x, m_calibOffset.force.y, m_calibOffset.force.z,
		  m_calibOffset.torque.x, m_calibOffset.torque.y, m_calibOffset.torque.z
		);
                offset_.force.x = m_calibOffset.force.x;
                offset_.force.y = m_calibOffset.force.y;
                offset_.force.z = m_calibOffset.force.z;
                offset_.torque.x= m_calibOffset.torque.x;
                offset_.torque.y = m_calibOffset.torque.y;
                offset_.torque.z = m_calibOffset.torque.z;
            }
            else
            {
                ROS_INFO("Calibrating sensor. Plase wait...");
                geometry_msgs::Wrench temp_offset;
                if (calibrate(true, &temp_offset))
                {
                    success = false;
                    msg = "Calibration failed! :/";
                }
            }

            m_isInitialized = true;
            success = true;
            msg = "FTS initalised!";

            apply_offset = true;

            // start timer for reading FT-data
            ftUpdateTimer_.start();
            ftPullTimer_.start();


        }
        else
        {
            m_isInitialized = false;
            success = false;
            msg = "FTS could not be initilised! :/";
        }
    }
}

bool ForceTorqueSensor::srvCallback_Init(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    std::string msg;
    bool success;

    init_sensor(msg, success);
    res.message = msg;
    res.success = success;

    return true;
}

bool ForceTorqueSensor::srvCallback_CalculateAverageMasurement(ati_force_torque::CalculateAverageMasurement::Request& req, ati_force_torque::CalculateAverageMasurement::Response& res)
{
    if (m_isInitialized)
    {
        res.success = true;
        res.message = "Measurement successfull! :)";
        res.measurement = makeAverageMeasurement(req.N_measurements, req.T_between_meas, req.frame_id);
    }
    else
    {
        res.success = false;
        res.message = "FTS not initialised! :/";
    }

    return true;
}

bool ForceTorqueSensor::srvCallback_CalculateOffset(ati_force_torque::CalculateSensorOffset::Request& req, ati_force_torque::CalculateSensorOffset::Response& res)
{
    if (m_isInitialized)
    {
        if (calibrate(req.apply_after_calculation, &res.offset))
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

bool ForceTorqueSensor::srvCallback_recalibrate(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
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
    tf2::doTransform(gravity, gravity_transformed,
                     p_tfBuffer->lookupTransform(sensor_frame_, transform_frame_, ros::Time(0)));
    geometry_msgs::Wrench offset;
    calibrate(false, &offset);
    offset_.force.x -= gravity_transformed.vector.x;
    offset_.force.y -= gravity_transformed.vector.y;
    offset_.force.z -= gravity_transformed.vector.z;
    offset_.torque.x -= (gravity_transformed.vector.y * cog.z - gravity_transformed.vector.z * cog.y);
    offset_.torque.y -= (gravity_transformed.vector.z * cog.x - gravity_transformed.vector.x * cog.z);
    offset_.torque.z -= (gravity_transformed.vector.x * cog.y - gravity_transformed.vector.y * cog.x);
    res.success = true;
    res.message = "Successfully recalibrated FTS!";
    return true;
}

bool ForceTorqueSensor::calibrate(bool apply_after_calculation, geometry_msgs::Wrench *new_offset)
{
    apply_offset = false;

    ROS_INFO("Calibrating using %d measurements and %f s pause between measurements.", calibrationNMeasurements, calibrationTBetween);
    geometry_msgs::Wrench temp_offset = makeAverageMeasurement(calibrationNMeasurements, calibrationTBetween);

    apply_offset = true;

    if (apply_after_calculation) {
        offset_ = temp_offset;
    }
    ROS_INFO("Calibration Data: Fx: %f; Fy: %f; Fz: %f; Mx: %f; My: %f; Mz: %f", temp_offset.force.x, temp_offset.force.y, temp_offset.force.z, temp_offset.torque.x, temp_offset.torque.y, temp_offset.torque.z);

    m_isCalibrated = true;
    *new_offset = temp_offset;

    return m_isCalibrated;
}

geometry_msgs::Wrench ForceTorqueSensor::makeAverageMeasurement(uint number_of_measurements, double time_between_meas, std::string frame_id)
{
    geometry_msgs::Wrench measurement;
    int num_of_errors = 0;

    ros::Duration duration(time_between_meas);

    for (int i = 0; i < number_of_measurements; i++)
    {
      geometry_msgs::Wrench output;
      
      if (frame_id.compare("") != 0) {  
	if (not transform_wrench(frame_id, sensor_frame_, moving_mean_filtered_wrench.wrench, &output))
	{
	  num_of_errors++;
	  if (num_of_errors > 200){
	    return measurement;
	  }
	  i--;
	  continue;
	}
      }
      else
      {
	output = moving_mean_filtered_wrench.wrench;
      }
    
      measurement.force.x += output.force.x;
      measurement.force.y += output.force.y;
      measurement.force.z += output.force.z;
      measurement.torque.x += output.torque.x;
      measurement.torque.y += output.torque.y;
      measurement.torque.z+= output.torque.z;

      duration.sleep();
    }
    measurement.force.x /= number_of_measurements;
    measurement.force.y /= number_of_measurements;
    measurement.force.z /= number_of_measurements;
    measurement.torque.x /= number_of_measurements;
    measurement.torque.y /= number_of_measurements;
    measurement.torque.z /= number_of_measurements;

    return measurement;
}


// TODO: make this to use filtered data (see calibrate)
bool ForceTorqueSensor::srvCallback_DetermineCoordinateSystem(std_srvs::Trigger::Request& req,
                                                              std_srvs::Trigger::Response& res)
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

bool ForceTorqueSensor::srvReadDiagnosticVoltages(ati_force_torque::DiagnosticVoltages::Request& req,
                                                  ati_force_torque::DiagnosticVoltages::Response& res)
{
    p_Ftc->ReadDiagnosticADCVoltages(req.index, res.adc_value);

    return true;
}

void ForceTorqueSensor::pullFTData(const ros::TimerEvent &event)
{
    int status = 0;

    bool bRet = p_Ftc->ReadSGData(status, sensor_data.wrench.force.x, sensor_data.wrench.force.y, sensor_data.wrench.force.z,
                                                        sensor_data.wrench.torque.x, sensor_data.wrench.torque.y, sensor_data.wrench.torque.z);
    if (bRet != false)
    {
        sensor_data.header.stamp = ros::Time::now();
        sensor_data.header.frame_id = sensor_frame_;

        if (apply_offset) {
            sensor_data.wrench.force.x  -= offset_.force.x;
            sensor_data.wrench.force.y  -= offset_.force.y;
            sensor_data.wrench.force.z  -= offset_.force.z;
            sensor_data.wrench.torque.x -= offset_.torque.x;
            sensor_data.wrench.torque.y -= offset_.torque.y;
            sensor_data.wrench.torque.z -= offset_.torque.z;
        }

        //lowpass
        low_pass_filtered_data.header = sensor_data.header;
        low_pass_filtered_data.wrench.force.x = lp_filter_force_x_.applyFilter(sensor_data.wrench.force.x);
        low_pass_filtered_data.wrench.force.y = lp_filter_force_y_.applyFilter(sensor_data.wrench.force.y);
        low_pass_filtered_data.wrench.force.z = lp_filter_force_z_.applyFilter(sensor_data.wrench.force.z);
        low_pass_filtered_data.wrench.torque.x = lp_filter_torque_x_.applyFilter(sensor_data.wrench.torque.x);
        low_pass_filtered_data.wrench.torque.y = lp_filter_torque_y_.applyFilter(sensor_data.wrench.torque.y);
        low_pass_filtered_data.wrench.torque.z = lp_filter_torque_z_.applyFilter(sensor_data.wrench.torque.z);

        //moving_mean
        moving_mean_filtered_wrench.header = low_pass_filtered_data.header;
        moving_mean_filtered_wrench.wrench.force.x = moving_mean_filter_force_x_.applyFilter(low_pass_filtered_data.wrench.force.x);
        moving_mean_filtered_wrench.wrench.force.y = moving_mean_filter_force_y_.applyFilter(low_pass_filtered_data.wrench.force.y);
        moving_mean_filtered_wrench.wrench.force.z = moving_mean_filter_force_z_.applyFilter(low_pass_filtered_data.wrench.force.z);
        moving_mean_filtered_wrench.wrench.torque.x = moving_mean_filter_torque_x_.applyFilter(low_pass_filtered_data.wrench.torque.x);
        moving_mean_filtered_wrench.wrench.torque.y = moving_mean_filter_torque_y_.applyFilter(low_pass_filtered_data.wrench.torque.y);
        moving_mean_filtered_wrench.wrench.torque.z = moving_mean_filter_torque_z_.applyFilter(low_pass_filtered_data.wrench.torque.z);


        if(is_pub_sensor_data_)
            sensor_data_pub_.publish(sensor_data);
        if(is_pub_low_pass_)
            low_pass_pub_.publish(low_pass_filtered_data);
	if(is_pub_moving_mean_)
            moving_mean_pub_.publish(moving_mean_filtered_wrench);
    }
}

void ForceTorqueSensor::filterFTData(){
  
  
    transformed_data.header.stamp = moving_mean_filtered_wrench.header.stamp;
    transformed_data.header.frame_id = transform_frame_;
    if (transform_wrench(transform_frame_, sensor_frame_, moving_mean_filtered_wrench.wrench, &transformed_data.wrench))
    {
      //gravity compensation
      gravity_compensated_force = gravity_compensator_.compensate(transformed_data);

      //treshhold filtering
      threshold_filtered_force = threshold_filter_.applyFilter(gravity_compensated_force);

      if(is_pub_transformed_data_)
	transformed_data_pub_.publish(transformed_data);
      if(is_pub_gravity_compensated_)
	gravity_compensated_pub_.publish(gravity_compensated_force);
      if(is_pub_threshold_filtered_)
	threshold_filtered_pub_.publish(threshold_filtered_force);
    }
}

bool ForceTorqueSensor::transform_wrench(std::string goal_frame, std::string source_frame, geometry_msgs::Wrench wrench, geometry_msgs::Wrench *transformed)
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

