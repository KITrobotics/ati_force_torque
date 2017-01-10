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
    srvServer_Calibrate_ = nh_.advertiseService("Calibrate", &ForceTorqueSensor::srvCallback_Calibrate, this);
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

    //Wrench Publisher
    nh_.param<bool>("Publish/gravity_compensated", is_pub_gravity_compensated_, false);
    if(is_pub_gravity_compensated_){
        gravity_compensated_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("gravity_compensated", 1);
    }
    nh_.param<bool>("Publish/low_pass", is_pub_low_pass_, false);
    if(is_pub_low_pass_){
        low_pass_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("low_pass", 1);
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
            F_avg.resize(6);
            if (m_staticCalibration)
            {
                ROS_INFO("Using static calibration from paramter server.");
                F_avg[0] = m_calibOffset.force.x;
                F_avg[1] = m_calibOffset.force.y;
                F_avg[2] = m_calibOffset.force.z;
                F_avg[3] = m_calibOffset.torque.x;
                F_avg[4] = m_calibOffset.torque.y;
                F_avg[5] = m_calibOffset.torque.z;
            }
            else
            {
                ROS_INFO("Calibrating sensor. Plase wait...");
                if (calibrate())
                {
                    success = false;
                    msg = "Calibration failed! :/";
                }
            }

            m_isInitialized = true;
            success = true;
            msg = "FTS initalised!";

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

bool ForceTorqueSensor::srvCallback_Calibrate(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
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

bool ForceTorqueSensor::calibrate()
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
    double Fx, Fy, Fz, Tx, Ty, Tz = 0;

    bool bRet = p_Ftc->ReadSGData(status, Fx, Fy, Fz, Tx, Ty, Tz);
    if (bRet != false)
    {
        sensor_data.header.stamp = ros::Time::now();
        sensor_data.header.frame_id = sensor_frame_;

        sensor_data.wrench.force.x  = Fx - F_avg[0];
        sensor_data.wrench.force.y  = Fy - F_avg[1];
        sensor_data.wrench.force.z  = Fz - F_avg[2];
        sensor_data.wrench.torque.x = Tx - F_avg[3];
        sensor_data.wrench.torque.y = Ty - F_avg[4];
        sensor_data.wrench.torque.z = Tz - F_avg[5];

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
    }
}

void ForceTorqueSensor::filterFTData(){

    try
    {
        transform_ee_base_stamped = p_tfBuffer->lookupTransform(transform_frame_, sensor_frame_, ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    geometry_msgs::Vector3Stamped temp_vector_in, temp_vector_out;

    transformed_data.header.stamp = sensor_data.header.stamp;
    transformed_data.header.frame_id = transform_frame_;

    temp_vector_in.vector = moving_mean_filtered_wrench.wrench.force;
    tf2::doTransform(temp_vector_in, temp_vector_out, transform_ee_base_stamped);
    transformed_data.wrench.force = temp_vector_out.vector;

    temp_vector_in.vector = moving_mean_filtered_wrench.wrench.torque;
    tf2::doTransform(temp_vector_in, temp_vector_out, transform_ee_base_stamped);
    transformed_data.wrench.torque = temp_vector_out.vector;


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

