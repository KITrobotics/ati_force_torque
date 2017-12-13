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

// general includes
#include <ati_force_torque/ati_force_torque_hw_can.h>

#include <pluginlib/class_list_macros.h>

// Headrs provided by cob-packages
//#include <cob_generic_can/CanESD.h>
//#include <cob_generic_can/CanPeakSys.h>
#include <cob_generic_can/CanPeakSysUSB.h>
#include <cob_generic_can/SocketCan.h>

ATIForceTorqueSensorHWCan::ATIForceTorqueSensorHWCan()
{
  m_pCanCtrl = NULL;

  // for types and baudrates see:
  // https://github.com/ipa320/cob_robots/blob/hydro_dev/cob_hardware_config/raw3-5/config/base/CanCtrl.ini
  m_CanType = CANITFTYPE_CAN_PEAK_USB;
  m_CanDevice = "/dev/pcan32";
  m_CanBaudrate = CANITFBAUD_250K;
  m_CanBaseIdentifier = 0x20 << 4;
}

ATIForceTorqueSensorHWCan::ATIForceTorqueSensorHWCan(int can_type, std::string can_path, int can_baudrate, int base_identifier)
{
  m_pCanCtrl = NULL;

  // for types and baudrates see:
  // https://github.com/ipa320/cob_robots/blob/hydro_dev/cob_hardware_config/raw3-5/config/base/CanCtrl.ini
  m_CanType = can_type;
  m_CanDevice = can_path;
  m_CanBaudrate = can_baudrate;
  m_CanBaseIdentifier = base_identifier << 4;
}

ATIForceTorqueSensorHWCan::~ATIForceTorqueSensorHWCan()
{
  if (m_pCanCtrl != NULL)
  {
    delete m_pCanCtrl;
  }
}

bool ATIForceTorqueSensorHWCan::initCommunication(int can_type, std::string can_path, int can_baudrate, int base_identifier)
{
  m_pCanCtrl = NULL;

  // for types and baudrates see:
  // https://github.com/ipa320/cob_robots/blob/hydro_dev/cob_hardware_config/raw3-5/config/base/CanCtrl.ini
  m_CanType = can_type;
  m_CanDevice = can_path;
  m_CanBaudrate = can_baudrate;
  m_CanBaseIdentifier = base_identifier << 4;
}

bool ATIForceTorqueSensorHWCan::init()
{
  bool ret = true;

  if (initCan())
  {
    // This is way of testig if communication is also successful
    if (!ReadFTSerialNumber())
    {
      std::cout << "Can not read Serial Number from FTS!" << std::endl;
      ret = false;
    }
    if (!ReadFirmwareVersion())
    {
      std::cout << "Can not read Firmware version from FTS!" << std::endl;
      ret = false;
    }
    if (!ReadCountsPerUnit())
    {
      std::cout << "Can not read Counts Per Unit from FTS!" << std::endl;
      ret = false;
    }
    if (!ReadUnitCodes())
    {
      std::cout << "Can not read Unit Codes from FTS!" << std::endl;
      ret = false;
    }
    // Add return values and checking
    SetActiveCalibrationMatrix(0);
    ReadCalibrationMatrix();
  }
  else
  {
    std::cout << "CAN initialisation unsuccessful!" << std::endl;
    ret = false;
  }

  return ret;
}

bool ATIForceTorqueSensorHWCan::initCan()
{
  bool ret = true;

  // current implementation only for CanPeakSysUSB and SocketCan
  switch (m_CanType)
  {
    case CANITFTYPE_CAN_PEAK_USB:
      m_pCanCtrl = new CANPeakSysUSB(m_CanDevice.c_str(), m_CanBaudrate);
      ret = m_pCanCtrl->init_ret();
      break;

    case CANITFTYPE_SOCKET_CAN:
      m_pCanCtrl = new SocketCan(m_CanDevice.c_str());
      ret = m_pCanCtrl->init_ret();
      break;
  }
  return ret;
}

bool ATIForceTorqueSensorHWCan::ReadFTSerialNumber()
{
#if DEBUG
  std::cout << "\n\n*********FTSerialNumber**********" << std::endl;
#endif
  bool ret = true;
  CanMsg CMsg;
  CMsg.setID(m_CanBaseIdentifier | READ_SERIALNR);
  CMsg.setLength(0);
  ret = m_pCanCtrl->transmitMsg(CMsg, true);
  if (ret)
  {
    CanMsg replyMsg;
    replyMsg.set(0, 0, 0, 0, 0, 0, 0, 0);
    ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);

    if (ret)
    {
#if DEBUG
      std::cout << "reply ID: \t" << std::hex << replyMsg.getID() << std::endl;
      std::cout << "reply Length: \t" << replyMsg.getLength() << std::endl;
#endif
      if (replyMsg.getID() == (m_CanBaseIdentifier | READ_SERIALNR))
      {
#if DEBUG
        std::cout << "Reading Serial Number Succeeded!" << std::endl;
        std::cout << "reply Data: \t" << (char)replyMsg.getAt(0) << " " << (char)replyMsg.getAt(1) << " "
                  << (char)replyMsg.getAt(2) << " " << (char)replyMsg.getAt(3) << " " << (char)replyMsg.getAt(4) << " "
                  << (char)replyMsg.getAt(5) << " " << (char)replyMsg.getAt(6) << " " << (char)replyMsg.getAt(7)
                  << std::endl;
#endif
      }
      else
      {
#if DEBUG
        std::cout << "Error: Received wrong opcode!" << std::endl;
#endif
        ret = false;
      }
    }
    else
    {
#if DEBUG
      std::cout << "ATIForceTorqueSensorHWCan::ReadFTSerialNumber(): Can not read message!" << std::endl;
#endif
    }
  }
  else
  {
#if DEBUG
    std::cout << "ATIForceTorqueSensorHWCan::ReadFTSerialNumber(): Can not transmit message!" << std::endl;
#endif
  }

  return ret;
}

bool ATIForceTorqueSensorHWCan::ReadCountsPerUnit()
{
#if DEBUG
  std::cout << "\n\n*********Read Counts Per Unit**********" << std::endl;
#endif
  bool ret = true;
  float countsPerForce = 0, countsPerTorque = 0;
  CanMsg CMsg;
  CMsg.setID(m_CanBaseIdentifier | READ_COUNTSPERU);
  CMsg.setLength(0);

  ret = m_pCanCtrl->transmitMsg(CMsg, true);

  if (ret)
  {
    CanMsg replyMsg;
    ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);

    if (ret)
    {
#if DEBUG
      std::cout << "reply ID: \t" << std::hex << replyMsg.getID() << std::endl;
      std::cout << "reply Length: \t" << replyMsg.getLength() << std::endl;
#endif
      if (replyMsg.getID() == (m_CanBaseIdentifier | READ_COUNTSPERU))
      {
#if DEBUG
        std::cout << "Reading Counts Per Unit Succeeded!" << std::endl;
        std::cout << "reply Data: \t" << (char)replyMsg.getAt(0) << " " << (char)replyMsg.getAt(1) << " "
                  << (char)replyMsg.getAt(2) << " " << (char)replyMsg.getAt(3) << " " << (char)replyMsg.getAt(4) << " "
                  << (char)replyMsg.getAt(5) << " " << (char)replyMsg.getAt(6) << " " << (char)replyMsg.getAt(7)
                  << std::endl;
#endif
      }
      else
      {
#if DEBUG
        std::cout << "Error: Received wrong opcode!" << std::endl;
#endif
        ret = false;
      }

      intbBuf.bytes[0] = replyMsg.getAt(3);
      intbBuf.bytes[1] = replyMsg.getAt(2);
      intbBuf.bytes[2] = replyMsg.getAt(1);
      intbBuf.bytes[3] = replyMsg.getAt(0);
      countsPerForce = intbBuf.value;

      intbBuf.bytes[0] = replyMsg.getAt(7);
      intbBuf.bytes[1] = replyMsg.getAt(6);
      intbBuf.bytes[2] = replyMsg.getAt(5);
      intbBuf.bytes[3] = replyMsg.getAt(4);
      countsPerTorque = intbBuf.value;
    }
    else
    {
      std::cout << "ATIForceTorqueSensorHWCan::ReadCountsPerUnit(): Can not read message!" << std::endl;
    }
  }
  else
  {
    std::cout << "ATIForceTorqueSensorHWCan::ReadCountsPerUnit(): Can not transmit message!" << std::endl;
  }
#if DEBUG
  std::cout << "CountsPerforce: " << countsPerForce << "  CountsPerTorque: " << countsPerTorque << std::endl;
#endif
  return ret;
}

bool ATIForceTorqueSensorHWCan::ReadUnitCodes()
{
#if DEBUG
  std::cout << "\n\n*********Read Unit Codes**********" << std::endl;
#endif
  bool ret = true;
  CanMsg CMsg;
  CMsg.setID(m_CanBaseIdentifier | READ_UNITCODE);
  CMsg.setLength(0);

  ret = m_pCanCtrl->transmitMsg(CMsg, true);

  if (ret)
  {
    CanMsg replyMsg;
    ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);

    if (ret)
    {
#if DEBUG
      std::cout << "reply ID: \t" << std::hex << replyMsg.getID() << std::endl;
      std::cout << "reply Length: \t" << replyMsg.getLength() << std::endl;
#endif
      if (replyMsg.getID() == (m_CanBaseIdentifier | READ_UNITCODE))
      {
#if DEBUG
        std::cout << "Reading Unit codes Succeed!" << std::endl;
        std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << std::endl;
#endif
      }
    }
    else
    {
      std::cout << "ATIForceTorqueSensorHWCan::ReadUnitCodes(): Can not read message!" << std::endl;
    }
  }
  else
  {
    std::cout << "ATIForceTorqueSensorHWCan::ReadUnitCodes(): Can not transmit message!" << std::endl;
  }

  return ret;
}

bool ATIForceTorqueSensorHWCan::readDiagnosticADCVoltages(int index, short int &value)
{
  // TODO: Check for Init
#if DEBUG
  std::cout << "\n\n*******Read Diagnostic ADC Voltages on index: " << index << "********" << std::endl;
#endif
  bool ret = true;
  CanMsg CMsg;
  CMsg.setID(m_CanBaseIdentifier | READ_DIAGNOV);
  CMsg.setLength(1);
  // TODO: offset or typecast index
  CMsg.setAt(index, 0);

  ret = m_pCanCtrl->transmitMsg(CMsg, true);

  if (ret)
  {
    CanMsg replyMsg;
    ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
    if (ret)
    {
#if DEBUG
      std::cout << "reply ID: \t" << std::hex << replyMsg.getID() << std::endl;
      std::cout << "reply Length: \t" << replyMsg.getLength() << std::endl;
#endif
      if (replyMsg.getID() == (m_CanBaseIdentifier | READ_DIAGNOV))
      {
#if DEBUG
        std::cout << "Reading Diagnostic ADC Voltage succeed!" << std::endl;
        std::cout << "ADC Voltage of diagnostic value " << index << " : " << replyMsg.getAt(0) << " "
                  << replyMsg.getAt(1) << std::endl;
#endif

        ibBuf.bytes[0] = replyMsg.getAt(1);
        ibBuf.bytes[1] = replyMsg.getAt(0);
        value = ibBuf.value;
      }
      else
        std::cout << "Error: Received wrong opcode!" << std::endl;
    }
    else
      std::cout << "Error: Receiving Message failed!" << std::endl;
  }
  else
  {
    std::cout << "ATIForceTorqueSensorHWCan::readDiagnosticADCVoltages(byte index): Can not transmit message!" << std::endl;
  }

  return ret;
}

bool ATIForceTorqueSensorHWCan::SetActiveCalibrationMatrix(int num)
{
#if DEBUG
  std::cout << "\n\n*******Setting Active Calibration Matrix Num to: " << num << "********" << std::endl;
#endif
  bool ret = true;
  CanMsg CMsg;
  CMsg.setID(m_CanBaseIdentifier | SET_CALIB);
  CMsg.setLength(1);
  CMsg.setAt(num, 0);

  ret = m_pCanCtrl->transmitMsg(CMsg, true);

  if (ret)
  {
    CanMsg replyMsg;
    ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
    if (ret)
    {
#if DEBUG
      std::cout << "reply ID: \t" << std::hex << replyMsg.getID() << std::endl;
      std::cout << "reply Length: \t" << replyMsg.getLength() << std::endl;
#endif
      if (replyMsg.getID() == (m_CanBaseIdentifier | SET_CALIB))
      {
#if DEBUG
        std::cout << "Setting Calibration Matrix succeed!" << std::endl;
        std::cout << "Calibration Matrix: " << replyMsg.getAt(0) << " is Activ!" << std::endl;
#endif
      }
      else
      {
#if DEBUG
        std::cout << "Error: Received wrong opcode!" << std::endl;
#endif
        ret = false;
      }
    }
    else
    {
      std::cout << "Error: Receiving Message failed!" << std::endl;
      ret = false;
    }
  }
  else
  {
    std::cout << "ATIForceTorqueSensorHWCan::SetActiveCalibrationMatrix(int num): Can not transmit message!" << std::endl;
    ret = false;
  }

  return ret;
}

bool ATIForceTorqueSensorHWCan::SetBaudRate(int value)
{
#if DEBUG
  std::cout << "\n\n*******Setting Baud Rate value to: " << value << "********" << std::endl;
#endif
  bool ret = true;
  CanMsg CMsg;
  CMsg.setID(m_CanBaseIdentifier | SET_BAUD);
  CMsg.setLength(1);
  CMsg.setAt(value, 0);

  ret = m_pCanCtrl->transmitMsg(CMsg, true);

  if (ret)
  {
    CanMsg replyMsg;
    ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
    if (ret)
    {
#if DEBUG
      std::cout << "reply ID: \t" << std::hex << replyMsg.getID() << std::endl;
      std::cout << "reply Length: \t" << replyMsg.getLength() << std::endl;
#endif
      if (replyMsg.getID() == (m_CanBaseIdentifier | SET_BAUD))
      {
#if DEBUG
        std::cout << "Send Baud Rate value: " << CMsg.getAt(0) << "!" << std::endl;
        std::cout << "Setting Baud Rate succeed!" << std::endl;
#endif
      }
      else
      {
        std::cout << "Error: Received wrong opcode!" << std::endl;
        ret = false;
      }
    }
    else
    {
      std::cout << "Error: Receiving Message failed!" << std::endl;
      ret = false;
    }
  }
  else
  {
    std::cout << "ATIForceTorqueSensorHWCan::SetBaudRate(int value): Can not transmit message!" << std::endl;
    ret = false;
  }

  return ret;
}

bool ATIForceTorqueSensorHWCan::Reset()
{
  std::cout << "\n\n******* Reseting the NETCANOEM ********" << std::endl;
  bool ret = true;
  CanMsg CMsg;
  CMsg.setID(m_CanBaseIdentifier | RESET);
  CMsg.setLength(0);

  ret = m_pCanCtrl->transmitMsg(CMsg, true);

  if (!ret)
  {
    std::cout << "ATIForceTorqueSensorHWCan::Reset(): Can not transmit message!" << std::endl;
    ret = false;
  }

  usleep(10000);

  return ret;
}

bool ATIForceTorqueSensorHWCan::SetBaseIdentifier(int identifier)
{
#if DEBUG
  std::cout << "\n\n*******Setting Base Identifier value to HEX : " << std::hex << identifier << " ********"
            << std::endl;
#endif
  bool ret = true;
  CanMsg CMsg;
  CMsg.setID(m_CanBaseIdentifier | SET_BASEID);
  CMsg.setLength(1);
  CMsg.setAt(identifier, 0);

  ret = m_pCanCtrl->transmitMsg(CMsg, true);

  if (ret)
  {
    CanMsg replyMsg;
    ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
    if (ret)
    {
#if DEBUG
      std::cout << "reply ID: \t" << std::hex << replyMsg.getID() << std::endl;
      std::cout << "reply Length: \t" << replyMsg.getLength() << std::endl;
#endif
      if (replyMsg.getID() == (m_CanBaseIdentifier | SET_BASEID))
      {
#if DEBUG
        std::cout << "Setting Base Identifier succeed!" << std::endl;
        std::cout << "Send Base Identifier value: " << std::hex << CMsg.getAt(0) << "!" << std::endl;
#endif
        ret = true;
      }
      else
      {
        std::cout << "Error: Received wrong opcode!" << std::endl;
        ret = false;
      }
    }
    else
    {
      std::cout << "Error: Receiving Message failed!" << std::endl;
      ret = false;
    }
  }
  else
  {
    std::cout << "ATIForceTorqueSensorHWCan::SetBaseIdentifier(int identifier): Can not transmit message!" << std::endl;
    ret = false;
  }

  return ret;
}

void ATIForceTorqueSensorHWCan::ReadCalibrationMatrix()
{
  Eigen::VectorXf vCoef(6);

  // Read Fx coefficients
  ReadMatrix(0, vCoef);
  m_v3FXGain = vCoef;

  // Read Fy coefficients
  ReadMatrix(1, vCoef);
  m_v3FYGain = vCoef;

  // Read Fz coefficients
  ReadMatrix(2, vCoef);
  m_v3FZGain = vCoef;

  // Read Tx coefficients
  ReadMatrix(3, vCoef);
  m_v3TXGain = vCoef;

  // Read Ty coefficients
  ReadMatrix(4, vCoef);
  m_v3TYGain = vCoef;

  // Read Tz coefficients
  ReadMatrix(5, vCoef);
  m_v3TZGain = vCoef;
  SetCalibMatrix();
}

void ATIForceTorqueSensorHWCan::ReadMatrix(int axis, Eigen::VectorXf &vec)
{
#if DEBUG
  std::cout << "\n\n*******Read Matrix**********" << std::endl;
#endif
  float statusCode = 0, sg0 = 0.0, sg1 = 0.0, sg2 = 0.0, sg3 = 0.0, sg4 = 0.0, sg5 = 0.0;

  CanMsg CMsg;
  CMsg.setID(m_CanBaseIdentifier | READ_MATRIX);
  CMsg.setLength(1);
  CMsg.setAt(axis, 0);

  bool ret = m_pCanCtrl->transmitMsg(CMsg, true);
  if (!ret)
  {
    std::cout << "Error: Requesting Calibration Matrix!" << std::endl;
    return;
  }

  CanMsg replyMsg;
  bool ret2 = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
  if (ret2)
  {
#if DEBUG
    std::cout << "reply ID: \t" << std::hex << replyMsg.getID() << std::endl;
    std::cout << "reply Length: \t" << replyMsg.getLength() << std::endl;
#endif
    if (replyMsg.getID() == (m_CanBaseIdentifier | READ_MATRIX))
    {
#if DEBUG
      std::cout << "Reading Matrix Succeeded!" << std::endl;
      std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " " << replyMsg.getAt(2) << " "
                << replyMsg.getAt(3) << " " << replyMsg.getAt(4) << " " << replyMsg.getAt(5) << " " << replyMsg.getAt(6)
                << " " << replyMsg.getAt(7) << std::endl;
#endif
    }
    else
    {
      std::cout << "Error: Received wrong opcode!" << std::endl;
      ret = false;
    }

    fbBuf.bytes[0] = replyMsg.getAt(3);
    fbBuf.bytes[1] = replyMsg.getAt(2);
    fbBuf.bytes[2] = replyMsg.getAt(1);
    fbBuf.bytes[3] = replyMsg.getAt(0);
    sg0 = fbBuf.value;

    fbBuf.bytes[0] = replyMsg.getAt(7);
    fbBuf.bytes[1] = replyMsg.getAt(6);
    fbBuf.bytes[2] = replyMsg.getAt(5);
    fbBuf.bytes[3] = replyMsg.getAt(4);
    sg1 = fbBuf.value;
  }
  else
    return;

  ret2 = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
  if (ret2)
  {
#if DEBUG
    std::cout << "reply ID: \t" << std::hex << replyMsg.getID() << std::endl;
    std::cout << "reply Length: \t" << replyMsg.getLength() << std::endl;
    std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " " << replyMsg.getAt(2) << " "
              << replyMsg.getAt(3) << " " << replyMsg.getAt(4) << " " << replyMsg.getAt(5) << " " << replyMsg.getAt(6)
              << " " << replyMsg.getAt(7) << std::endl;
#endif

    fbBuf.bytes[0] = replyMsg.getAt(3);
    fbBuf.bytes[1] = replyMsg.getAt(2);
    fbBuf.bytes[2] = replyMsg.getAt(1);
    fbBuf.bytes[3] = replyMsg.getAt(0);
    sg2 = fbBuf.value;

    fbBuf.bytes[0] = replyMsg.getAt(7);
    fbBuf.bytes[1] = replyMsg.getAt(6);
    fbBuf.bytes[2] = replyMsg.getAt(5);
    fbBuf.bytes[3] = replyMsg.getAt(4);
    sg3 = fbBuf.value;
  }
  else
    return;

  ret2 = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
  if (ret2)
  {
#if DEBUG
    std::cout << "reply ID: \t" << std::hex << replyMsg.getID() << std::endl;
    std::cout << "reply Length: \t" << replyMsg.getLength() << std::endl;
    std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " " << replyMsg.getAt(2) << " "
              << replyMsg.getAt(3) << " " << replyMsg.getAt(4) << " " << replyMsg.getAt(5) << " " << replyMsg.getAt(6)
              << " " << replyMsg.getAt(7) << std::endl;
#endif

    fbBuf.bytes[0] = replyMsg.getAt(3);
    fbBuf.bytes[1] = replyMsg.getAt(2);
    fbBuf.bytes[2] = replyMsg.getAt(1);
    fbBuf.bytes[3] = replyMsg.getAt(0);
    sg4 = fbBuf.value;

    fbBuf.bytes[0] = replyMsg.getAt(7);
    fbBuf.bytes[1] = replyMsg.getAt(6);
    fbBuf.bytes[2] = replyMsg.getAt(5);
    fbBuf.bytes[3] = replyMsg.getAt(4);
    sg5 = fbBuf.value;
  }
  else
    return;

  vec[0] = sg0;
  vec[1] = sg1;
  vec[2] = sg2;
  vec[3] = sg3;
  vec[4] = sg4;
  vec[5] = sg5;
#if DEBUG
  std::cout << "Matix:  SG0: " << sg0 << " SG1: " << sg1 << " SG2: " << sg2 << " SG3: " << sg3 << " SG4: " << sg4
            << " SG5: " << sg5 << std::endl;
#endif
}

bool ATIForceTorqueSensorHWCan::ReadFirmwareVersion()
{
#if DEBUG
  std::cout << "\n\n*******Reading Firmware Version*******" << std::endl;
#endif
  bool ret = true;
  CanMsg CMsg;
  CMsg.setID(m_CanBaseIdentifier | READ_FIRMWARE);
  CMsg.setLength(0);

  ret = m_pCanCtrl->transmitMsg(CMsg, true);

  if (ret)
  {
    CanMsg replyMsg;
    ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
    if (ret)
    {
#if DEBUG
      std::cout << "reply ID: \t" << std::hex << replyMsg.getID() << std::endl;
      std::cout << "reply Length: \t" << replyMsg.getLength() << std::endl;
#endif
      if (replyMsg.getID() == (m_CanBaseIdentifier | READ_FIRMWARE))
      {
#if DEBUG
        std::cout << "Reading Firmware Succeed!" << std::endl;
        std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " " << replyMsg.getAt(2)
                  << " " << replyMsg.getAt(3) << std::endl;
#endif
      }
      else
      {
        std::cout << "Error: Received wrong opcode!" << std::endl;
        ret = false;
      }
    }
    else
    {
      std::cout << "Error: Receiving Message failed!" << std::endl;
      ret = false;
    }
  }
  else
  {
    std::cout << "Error: Transmiting Message failed!" << std::endl;
    ret = false;
  }

  return ret;
}

bool ATIForceTorqueSensorHWCan::readFTData(int statusCode, double &Fx, double &Fy, double &Fz, double &Tx, double &Ty, double &Tz)
{
  int sg0 = 0, sg1 = 0, sg2 = 0, sg3 = 0, sg4 = 0, sg5 = 0;

  CanMsg CMsg;
  CMsg.setID(m_CanBaseIdentifier | READ_SG);
  CMsg.setLength(0);
  bool ret = m_pCanCtrl->transmitMsg(CMsg, true);

  if (!ret)
  {
    std::cout << "ATIForceTorqueSensorHWCan::ReadSGData: Error: Transmiting message failed!" << std::endl;
    return false;
  }

  CanMsg replyMsg;
  bool ret2 = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
  unsigned char c[2];
  if (ret2)
  {
    if (replyMsg.getID() != (m_CanBaseIdentifier | 0x0))
    {
      std::cout << "ATIForceTorqueSensorHWCan::ReadSGData: Error: Received wrong opcode (Should be 0x200)!" << std::endl;
      std::cout << "reply ID: \t" << std::hex << replyMsg.getID() << std::endl;
      std::cout << "reply Length: \t" << replyMsg.getLength() << std::endl;
      std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " " << replyMsg.getAt(2) << " "
                << replyMsg.getAt(3) << " " << replyMsg.getAt(4) << " " << replyMsg.getAt(5) << " " << replyMsg.getAt(6)
                << " " << replyMsg.getAt(7) << std::endl;
      return false ;
    }

    c[0] = replyMsg.getAt(0);  // status code
    c[1] = replyMsg.getAt(1);
    statusCode = (short)((c[0] << 8) | c[1]);

    if (statusCode != 0)
    {
      if (statusCode & 0x4000)
      {
        std::cout << "ATIForceTorqueSensorHWCan::ReadSGData: CAN bus error detected!" << std::endl;
        Reset();
        std::cout << "ATIForceTorqueSensorHWCan::ReadSGData: FTS reseted!" << std::endl;
      }
      else
      {
        std::cout << "ATIForceTorqueSensorHWCan::ReadSGData: Error: Something is wrong with sensor!" << std::endl;
        std::cout << std::hex << statusCode << std::endl;
      }
    }

    c[0] = replyMsg.getAt(2);  // sg0
    c[1] = replyMsg.getAt(3);
    sg0 = (short)((c[0] << 8) | c[1]);

    c[0] = replyMsg.getAt(4);  // sg2
    c[1] = replyMsg.getAt(5);
    sg2 = (short)((c[0] << 8) | c[1]);

    c[0] = replyMsg.getAt(6);  // sg4
    c[1] = replyMsg.getAt(7);
    sg4 = (short)((c[0] << 8) | c[1]);
  }
  else
    return false;

  ret2 = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
  if (ret2)
  {
    if (replyMsg.getID() != (m_CanBaseIdentifier | 0x1))
    {
      std::cout << "ATIForceTorqueSensorHWCan::ReadSGData: Error: Received wrong opcode (Should be 0x201)!" << std::endl;
      std::cout << "reply ID: \t" << std::hex << replyMsg.getID() << std::endl;
      std::cout << "reply Length: \t" << replyMsg.getLength() << std::endl;
      std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " " << replyMsg.getAt(2) << " "
                << replyMsg.getAt(3) << " " << replyMsg.getAt(4) << " " << replyMsg.getAt(5) << " " << replyMsg.getAt(6)
                << " " << replyMsg.getAt(7) << std::endl;
      return false ;
    }

    c[0] = replyMsg.getAt(0);  // sg1
    c[1] = replyMsg.getAt(1);
    sg1 = (short)((c[0] << 8) | c[1]);

    c[0] = replyMsg.getAt(2);  // sg3
    c[1] = replyMsg.getAt(3);
    sg3 = (short)((c[0] << 8) | c[1]);

    c[0] = replyMsg.getAt(4);  // sg5
    c[1] = replyMsg.getAt(5);
    sg5 = (short)((c[0] << 8) | c[1]);
  }
  else
    return false;

  StrainGaugeToForce(sg0, sg1, sg2, sg3, sg4, sg5);
  Fx = m_vForceData(0);
  Fy = m_vForceData(1);
  Fz = m_vForceData(2);
  Tx = m_vForceData(3);
  Ty = m_vForceData(4);
  Tz = m_vForceData(5);
  return true;
}

void ATIForceTorqueSensorHWCan::StrainGaugeToForce(int &sg0, int &sg1, int &sg2, int &sg3, int &sg4, int &sg5)
{
  Eigen::VectorXf v6SG(6);
  Eigen::VectorXf v6tmp(6);
  Eigen::VectorXf test(6);

  v6SG[0] = sg0;
  v6SG[1] = sg1;
  v6SG[2] = sg2;
  v6SG[3] = sg3;
  v6SG[4] = sg4;
  v6SG[5] = sg5;
  test = m_mXCalibMatrix * v6SG;
  m_vForceData = test * 0.000001;
}

void ATIForceTorqueSensorHWCan::SetGaugeOffset(float sg0Off, float sg1Off, float sg2Off, float sg3Off, float sg4Off, float sg5Off)
{
  Eigen::VectorXf tmp(6);
  tmp[0] = sg0Off;
  tmp[1] = sg1Off;
  tmp[2] = sg2Off;
  tmp[3] = sg3Off;
  tmp[4] = sg4Off;
  tmp[5] = sg5Off;
  m_v3StrainGaigeOffset = tmp;
}
void ATIForceTorqueSensorHWCan::SetGaugeGain(float gg0, float gg1, float gg2, float gg3, float gg4, float gg5)
{
  Eigen::VectorXf tmp(6);
  tmp[0] = gg0;
  tmp[1] = gg1;
  tmp[2] = gg2;
  tmp[3] = gg3;
  tmp[4] = gg4;
  tmp[5] = gg5;
  m_v3GaugeGain = tmp;
  // std::cout<<"GaugeGain: \n"<<m_v3GaugeGain<<"\n\n";
}

void ATIForceTorqueSensorHWCan::SetFXGain(float fxg0, float fxg1, float fxg2, float fxg3, float fxg4, float fxg5)
{
  Eigen::VectorXf tmp(6);
  tmp[0] = fxg0;
  tmp[1] = fxg1;
  tmp[2] = fxg2;
  tmp[3] = fxg3;
  tmp[4] = fxg4;
  tmp[5] = fxg5;
  m_v3FXGain = tmp;
  // std::cout<<"FXGain: \n"<<m_v3FXGain<<"\n\n";
}
void ATIForceTorqueSensorHWCan::SetFYGain(float fyg0, float fyg1, float fyg2, float fyg3, float fyg4, float fyg5)
{
  Eigen::VectorXf tmp(6);
  tmp[0] = fyg0;
  tmp[1] = fyg1;
  tmp[2] = fyg2;
  tmp[3] = fyg3;
  tmp[4] = fyg4;
  tmp[5] = fyg5;
  m_v3FYGain = tmp;
  // std::cout<<"FYGain: \n"<<m_v3FYGain<<"\n\n";
}
void ATIForceTorqueSensorHWCan::SetFZGain(float fzg0, float fzg1, float fzg2, float fzg3, float fzg4, float fzg5)
{
  Eigen::VectorXf tmp(6);
  tmp[0] = fzg0;
  tmp[1] = fzg1;
  tmp[2] = fzg2;
  tmp[3] = fzg3;
  tmp[4] = fzg4;
  tmp[5] = fzg5;
  m_v3FZGain = tmp;
  // std::cout<<"FZGain: \n"<<m_v3FZGain<<"\n\n";
}
void ATIForceTorqueSensorHWCan::SetTXGain(float txg0, float txg1, float txg2, float txg3, float txg4, float txg5)
{
  Eigen::VectorXf tmp(6);
  tmp[0] = txg0;
  tmp[1] = txg1;
  tmp[2] = txg2;
  tmp[3] = txg3;
  tmp[4] = txg4;
  tmp[5] = txg5;
  m_v3TXGain = tmp;
  // std::cout<<"TXGain: \n"<<m_v3TXGain<<"\n\n";
}
void ATIForceTorqueSensorHWCan::SetTYGain(float tyg0, float tyg1, float tyg2, float tyg3, float tyg4, float tyg5)
{
  Eigen::VectorXf tmp(6);
  tmp[0] = tyg0;
  tmp[1] = tyg1;
  tmp[2] = tyg2;
  tmp[3] = tyg3;
  tmp[4] = tyg4;
  tmp[5] = tyg5;
  m_v3TYGain = tmp;
  // std::cout<<"TYGain: \n"<<m_v3TYGain<<"\n\n";
}
void ATIForceTorqueSensorHWCan::SetTZGain(float tzg0, float tzg1, float tzg2, float tzg3, float tzg4, float tzg5)
{
  Eigen::VectorXf tmp(6);
  tmp[0] = tzg0;
  tmp[1] = tzg1;
  tmp[2] = tzg2;
  tmp[3] = tzg3;
  tmp[4] = tzg4;
  tmp[5] = tzg5;
  m_v3TZGain = tmp;
  // std::cout<<"TZGain: \n"<<m_v3TZGain<<"\n\n";
}

void ATIForceTorqueSensorHWCan::CalcCalibMatrix()
{
  Eigen::MatrixXf tmp(6, 6);
  tmp(0) = m_v3FXGain[0] / m_v3GaugeGain[0];
  tmp(1) = m_v3FXGain[1] / m_v3GaugeGain[1];
  tmp(2) = m_v3FXGain[2] / m_v3GaugeGain[2];
  tmp(3) = m_v3FXGain[3] / m_v3GaugeGain[3];
  tmp(4) = m_v3FXGain[4] / m_v3GaugeGain[4];
  tmp(5) = m_v3FXGain[5] / m_v3GaugeGain[5];

  tmp(6) = m_v3FYGain[0] / m_v3GaugeGain[0];
  tmp(7) = m_v3FYGain[1] / m_v3GaugeGain[1];
  tmp(8) = m_v3FYGain[2] / m_v3GaugeGain[2];
  tmp(9) = m_v3FYGain[3] / m_v3GaugeGain[3];
  tmp(10) = m_v3FYGain[4] / m_v3GaugeGain[4];
  tmp(11) = m_v3FYGain[5] / m_v3GaugeGain[5];

  tmp(12) = m_v3FZGain[0] / m_v3GaugeGain[0];
  tmp(13) = m_v3FZGain[1] / m_v3GaugeGain[1];
  tmp(14) = m_v3FZGain[2] / m_v3GaugeGain[2];
  tmp(15) = m_v3FZGain[3] / m_v3GaugeGain[3];
  tmp(16) = m_v3FZGain[4] / m_v3GaugeGain[4];
  tmp(17) = m_v3FZGain[5] / m_v3GaugeGain[5];

  tmp(18) = m_v3TXGain[0] / m_v3GaugeGain[0];
  tmp(19) = m_v3TXGain[1] / m_v3GaugeGain[1];
  tmp(20) = m_v3TXGain[2] / m_v3GaugeGain[2];
  tmp(21) = m_v3TXGain[3] / m_v3GaugeGain[3];
  tmp(22) = m_v3TXGain[4] / m_v3GaugeGain[4];
  tmp(23) = m_v3TXGain[5] / m_v3GaugeGain[5];

  tmp(24) = m_v3TYGain[0] / m_v3GaugeGain[0];
  tmp(25) = m_v3TYGain[1] / m_v3GaugeGain[1];
  tmp(26) = m_v3TYGain[2] / m_v3GaugeGain[2];
  tmp(27) = m_v3TYGain[3] / m_v3GaugeGain[3];
  tmp(28) = m_v3TYGain[4] / m_v3GaugeGain[4];
  tmp(29) = m_v3TYGain[5] / m_v3GaugeGain[5];

  tmp(30) = m_v3TZGain[0] / m_v3GaugeGain[0];
  tmp(31) = m_v3TZGain[1] / m_v3GaugeGain[1];
  tmp(32) = m_v3TZGain[2] / m_v3GaugeGain[2];
  tmp(33) = m_v3TZGain[3] / m_v3GaugeGain[3];
  tmp(34) = m_v3TZGain[4] / m_v3GaugeGain[4];
  tmp(35) = m_v3TZGain[5] / m_v3GaugeGain[5];

  m_mXCalibMatrix = tmp;
}

void ATIForceTorqueSensorHWCan::SetCalibMatrix()
{
  Eigen::MatrixXf tmp(6, 6);
  tmp(0) = m_v3FXGain[0];
  tmp(1) = m_v3FXGain[1];
  tmp(2) = m_v3FXGain[2];
  tmp(3) = m_v3FXGain[3];
  tmp(4) = m_v3FXGain[4];
  tmp(5) = m_v3FXGain[5];

  tmp(6) = m_v3FYGain[0];
  tmp(7) = m_v3FYGain[1];
  tmp(8) = m_v3FYGain[2];
  tmp(9) = m_v3FYGain[3];
  tmp(10) = m_v3FYGain[4];
  tmp(11) = m_v3FYGain[5];

  tmp(12) = m_v3FZGain[0];
  tmp(13) = m_v3FZGain[1];
  tmp(14) = m_v3FZGain[2];
  tmp(15) = m_v3FZGain[3];
  tmp(16) = m_v3FZGain[4];
  tmp(17) = m_v3FZGain[5];

  tmp(18) = m_v3TXGain[0];
  tmp(19) = m_v3TXGain[1];
  tmp(20) = m_v3TXGain[2];
  tmp(21) = m_v3TXGain[3];
  tmp(22) = m_v3TXGain[4];
  tmp(23) = m_v3TXGain[5];

  tmp(24) = m_v3TYGain[0];
  tmp(25) = m_v3TYGain[1];
  tmp(26) = m_v3TYGain[2];
  tmp(27) = m_v3TYGain[3];
  tmp(28) = m_v3TYGain[4];
  tmp(29) = m_v3TYGain[5];

  tmp(30) = m_v3TZGain[0];
  tmp(31) = m_v3TZGain[1];
  tmp(32) = m_v3TZGain[2];
  tmp(33) = m_v3TZGain[3];
  tmp(34) = m_v3TZGain[4];
  tmp(35) = m_v3TZGain[5];

  m_mXCalibMatrix = tmp.transpose();
}

PLUGINLIB_EXPORT_CLASS(ATIForceTorqueSensorHWCan, hardware_interface::ForceTorqueSensorHW)
