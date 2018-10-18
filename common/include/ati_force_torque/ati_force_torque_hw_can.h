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

#ifndef ATIFORCETORQUESENSORHWCAN_INCLUDEDEF_H
#define ATIFORCETORQUESENSORHWCAN_INCLUDEDEF_H

#include <iostream>
#include <fstream>
#include <Eigen/Core>

#include <force_torque_sensor/force_torque_sensor_hw.h>

// Headers provided by other cob-packages
#include <cob_generic_can/CanItf.h>

// opCodes for the ForceTorque Can Interface
// set as Can Message ID
#define READ_SG 0x0
#define READ_MATRIX 0x2
#define READ_SERIALNR 0x5
#define SET_CALIB 0x6
#define READ_COUNTSPERU 0x7
#define READ_UNITCODE 0x8
#define READ_DIAGNOV 0X9
#define RESET 0xC
#define SET_BASEID 0xD
#define SET_BAUD 0xE
#define READ_FIRMWARE 0xF

#define ATI_CAN_BAUD_2M 0
#define ATI_CAN_BAUD_1M 1
#define ATI_CAN_BAUD_500K 3
#define ATI_CAN_BAUD_250K 7

class ATIForceTorqueSensorHWCan : public hardware_interface::ForceTorqueSensorHW
{
public:
  ATIForceTorqueSensorHWCan();
  ATIForceTorqueSensorHWCan(int can_type, std::string can_path, int can_baudrate, int base_identifier);
  ~ATIForceTorqueSensorHWCan();

  bool init();
  bool initCommunication(int can_type, std::string can_path, int can_baudrate, int base_identifier);
  bool ReadFTSerialNumber();
  bool ReadCountsPerUnit();
  bool ReadUnitCodes();
  bool readDiagnosticADCVoltages(int index, short int& value);
  bool SetActiveCalibrationMatrix(int num);
  bool SetBaudRate(int value);
  bool SetBaseIdentifier(int identifier);
  bool Reset();
  bool readFTData(int statusCode, double& Fx, double& Fy, double& Fz, double& Tx, double& Ty, double& Tz);
  bool ReadFirmwareVersion();
  void ReadCalibrationMatrix();

  void SetGaugeOffset(float sg0Off, float sg1Off, float sg2Off, float sg3Off, float sg4Off, float sg5Off);
  void SetGaugeGain(float gg0, float gg1, float gg2, float gg3, float gg4, float gg5);
  void SetFXGain(float fxg0, float fxg1, float fxg2, float fxg3, float fxg4, float fxg5);
  void SetFYGain(float fyg0, float fyg1, float fyg2, float fyg3, float fyg4, float fyg5);
  void SetFZGain(float fzg0, float fzg1, float fzg2, float fzg3, float fzg4, float fzg5);
  void SetTXGain(float txg0, float txg1, float txg2, float txg3, float txg4, float txg5);
  void SetTYGain(float tyg0, float tyg1, float tyg2, float tyg3, float tyg4, float tyg5);
  void SetTZGain(float tzg0, float tzg1, float tzg2, float tzg3, float tzg4, float tzg5);

  void SetCalibMatrix();
  void CalcCalibMatrix();
  void StrainGaugeToForce(int& sg0, int& sg1, int& sg2, int& sg3, int& sg4, int& sg5);

protected:
  bool initCan();

  //--------------------------------- Variables
  CanMsg m_CanMsgRec;
  bool m_bWatchdogErr;

private:
  CanMsg CMsg;
  CanItf* m_pCanCtrl;

  int m_CanType;
  std::string m_CanDevice;
  int m_CanBaudrate;
  int m_CanBaseIdentifier;

  unsigned int d_len;
  Eigen::VectorXf m_v3StrainGaigeOffset;
  Eigen::VectorXf m_v3GaugeGain;
  Eigen::VectorXf m_v3FXGain;
  Eigen::VectorXf m_v3FYGain;
  Eigen::VectorXf m_v3FZGain;
  Eigen::VectorXf m_v3TXGain;
  Eigen::VectorXf m_v3TYGain;
  Eigen::VectorXf m_v3TZGain;
  Eigen::MatrixXf m_mXCalibMatrix;
  Eigen::MatrixXf m_vForceData;

  // the Parameter indicates the Axis row to Read
  // Fx = 0 | Fy = 1 | Fz = 2 | Tx = 3 | Ty = 4 | Tz = 5
  void ReadMatrix(int axis, Eigen::VectorXf& vec);

  union
  {
    char bytes[2];
    short int value;
  } ibBuf;

  union
  {
    char bytes[4];
    int value;
  } intbBuf;

  union
  {
    char bytes[4];
    float value;
  } fbBuf;

  std::ofstream out;
};

#endif
