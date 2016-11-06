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

ForceTorqueSensorHandle::ForceTorqueSensorHandle(ros::NodeHandle& nh, std::string sensor_name, std::string output_frame) :
    ForceTorqueSensor(nh), hardware_interface::ForceTorqueSensorHandle(sensor_name, output_frame, interface_force_, interface_torque_)
{
    transform_frame_ = output_frame;

}

void ForceTorqueSensorHandle::updateFTData(const ros::TimerEvent& event)
{
    filterFTData();
    interface_force_[0] = threshold_filtered_force.wrench.force.x;
    interface_force_[1] = threshold_filtered_force.wrench.force.y;
    interface_force_[2] = threshold_filtered_force.wrench.force.z;

    interface_torque_[0] = threshold_filtered_force.wrench.torque.x;
    interface_torque_[1] = threshold_filtered_force.wrench.torque.y;
    interface_torque_[2] = threshold_filtered_force.wrench.torque.z;
}
