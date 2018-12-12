/****************************************************************
 *
 * Copyright 2016 Intelligent Industrial Robotics (IIROB) Group,
 * Institute for Anthropomatics and Robotics (IAR) -
 * Intelligent Process Control and Robotics (IPR),
 * Karlsruhe Institute of Technology
 *
 * Author: Florian Aumann
 * Maintainer: Denis Štogl, email: denis.stogl@kit.edu
 *
 * Date of update: 2014-2018
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

#define DEBUG 0

// general includes
#include <ati_force_torque/ati_force_torque_hw_rs485.h>

#include <pluginlib/class_list_macros.h>

#include <unistd.h>
#include <iostream>
#include <sstream>
#include <fcntl.h>
#include <string>

#include <stropts.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <termios.h>
#include <unistd.h>
#include <arpa/inet.h>


ATIForceTorqueSensorHWRS485::ATIForceTorqueSensorHWRS485()
: m_modbusCtrl(0),
  m_rs485(0)
{
	// for baudrates see:
	// https://www.ati-ia.com/app_content/documents/9620-05-Digital%20FT.pdf
	m_RS485Device = "/dev/ttyUSB0";
	m_ModbusBaseIdentifier = 0xA;  			//ID of the ATI sensor is always set to 10 (As specified in 8.3)
	m_ModbusBaudrate = MODBUSBAUD_1250K;		///By default, the baudrate is set to 1.250.000
}

ATIForceTorqueSensorHWRS485::ATIForceTorqueSensorHWRS485(std::string device_path, int device_baudrate, int base_identifier)
: m_modbusCtrl(0),
  m_rs485(0)
{
	// for types and baudrates see:
	m_RS485Device = device_path;
	m_ModbusBaudrate = device_baudrate;
	m_ModbusBaseIdentifier = base_identifier;
}

ATIForceTorqueSensorHWRS485::~ATIForceTorqueSensorHWRS485()
{
	if (m_modbusCtrl != NULL)
	{
		if (m_isStreaming)
		{
			StopStreaming();
		}
		Close();
	}
}

bool ATIForceTorqueSensorHWRS485::initCommunication(int type, std::string path, int baudrate, int base_identifier)
{
	std::cout << "ATIForceTorqueSensorHWRS485::initCommunication" << std::endl;
	if (baudrate != MODBUSBAUD_1250K && baudrate != MODBUSBAUD_115200 && baudrate != MODBUSBAUD_19200)
	{
		#if DEBUG
		std::cout << "Baudrate " << baudrate << " is not supported!" << std::endl;
		#endif
		return false;
	}

	// for types and baudrates see:
	m_RS485Device = path;
	m_ModbusBaudrate = baudrate;
	m_ModbusBaseIdentifier = base_identifier;
	return true;
}

bool ATIForceTorqueSensorHWRS485::init()
{
#if DEBUG
  std::cout << "ATIForceTorqueSensorHWRS485::init" << std::endl;
#endif
  m_isStreaming = false;
  if (initRS485())
  {
	int tries = 0;
	while (!ReadFTCalibrationData(1) && tries < 10) {
		tries++;
		usleep(100000);
	}
    // This is a way of testing if communication is successful
    if (tries >= 10)
    {
      std::cout << "Can not read Calibration Data from FTS!" << std::endl;
      return false;
    }

    //Set active gain and offset from the calibration data
    if (!SetStorageLock(false))
    {
    	return false;
    }
    if (!SetActiveGain(m_calibrationData.gageGain[0], m_calibrationData.gageGain[1], m_calibrationData.gageGain[2], m_calibrationData.gageGain[3], m_calibrationData.gageGain[4], m_calibrationData.gageGain[5]))
    {
    	return false;
    }

    if (!SetActiveOffset(m_calibrationData.gageOffset[0], m_calibrationData.gageOffset[1], m_calibrationData.gageOffset[2], m_calibrationData.gageOffset[3], m_calibrationData.gageOffset[4], m_calibrationData.gageOffset[5]))
    {
    	return false;
    }

    if (!SetStorageLock(true))
    {
    	return false;
    }

    SetCalibMatrix(m_calibrationData.basicMatrix);
  }
  else
  {
    std::cout << "RS485 initialization unsuccessful!" << std::endl;
    return false;
  }

  return true;
}

bool ATIForceTorqueSensorHWRS485::initRS485()
{
#if DEBUG
  std::cout << "ATIForceTorqueSensorHWRS485::initRS485" << std::endl;
#endif
  if (m_modbusCtrl)
  {
	  Close();	//Close existing modbus connection before opening a new one
  }


#if DEBUG
  std::cout << "Opening new modbus connection to " << m_RS485Device.c_str() << " with baudrate " << m_ModbusBaudrate << std::endl;
#endif
  m_ModbusBaseIdentifier = 0x0A;

  int tries = 0;
  bool gotStatusWord = false;
  while (!gotStatusWord && tries < 10) {
	m_modbusCtrl = modbus_new_rtu(m_RS485Device.c_str(), m_ModbusBaudrate, 'E', 8, 1); //As specified in 8.1., the sensor uses 8 data bits and one bit for 'Even' parity
	if (!m_modbusCtrl)
	{
	  std::cout << "Could not initialize modbus connection" << std::endl;
	  tries++;
	  continue;
	}
	//#if DEBUG
	//  modbus_set_debug(m_modbusCtrl, true);
	//#endif
	#if DEBUG
	  std::cout << "Setting slave to " << m_ModbusBaseIdentifier << std::endl;
	#endif
	int rc = modbus_set_slave(m_modbusCtrl, m_ModbusBaseIdentifier);
	if (rc == -1)
	{
	  std::cout << "Could not set slave ID" << std::endl;
	  Close();
	  tries++;
	  continue;
	}

	rc = modbus_connect(m_modbusCtrl);
	if (rc == -1)
	{
	  std::cout << "Could not connect" << std::endl;
	  Close();
	  tries++;
	  continue;
	}
	// This is a way of testing if communication is successful
	gotStatusWord = ReadStatusWord();
	if(!gotStatusWord)
	{
      Close();
	  //Send stop sequence when starting a new connection, to make sure the sensor is not in streaming mode
	  if (!OpenRawConnection())
	  {
		  tries++;
		  continue;
	  }
	  SendStopSequence();
	  usleep(500000);
	}
    tries++;
  }

  if (!gotStatusWord)
  {
    std::cout << "Could not read status word!" << std::endl;
    return false;
  }
  m_isStreaming = false;
  return true;
}

bool ATIForceTorqueSensorHWRS485::ReadFTCalibrationData(const unsigned int calibrationNumber)
{
#if DEBUG
  std::cout << "\n\n*********FTSerialNumber**********" << std::endl;
#endif
  if (calibrationNumber < 1 || calibrationNumber > 16)
  {
	  std::cout << "Invalid calibration number" << std::endl;
	  return false;
  }

  uint16_t tab_reg[168];
  unsigned int registerOffset = (calibrationNumber - 1) * 192;
  uint8_t dest;

  int failure = false; 
  failure |= (modbus_read_registers(m_modbusCtrl, 0x00e3 + registerOffset, 64, tab_reg) == -1);
  failure |= (modbus_read_registers(m_modbusCtrl, 0x00e3 + registerOffset + 64, 64, &tab_reg[64]) == -1);
  failure |= (modbus_read_registers(m_modbusCtrl, 0x00e3 + registerOffset + 128, 40, &tab_reg[128]) == -1);

  if (failure)
  {
	  std::cout << "Reading Calibration Data failed" <<  std::endl;
      fprintf(stderr, "%s\n", modbus_strerror(errno));
      return false;
  }

  std::string calibSerialNumber;
  for (unsigned int i = 0; i < 4; i++)
  {
      calibSerialNumber += (char)MODBUS_GET_HIGH_BYTE(tab_reg[i]);
      calibSerialNumber += (char)MODBUS_GET_LOW_BYTE(tab_reg[i]);
  }
  m_calibrationData.calibSerialNumber = calibSerialNumber;
  #if DEBUG
  std::cout << "Serial Number is " << m_calibrationData.calibSerialNumber << std::endl;
#endif
  std::string calibPartNumber;
  for (unsigned int i = 4; i < 20; i++)
  {
      calibPartNumber += (char)MODBUS_GET_HIGH_BYTE(tab_reg[i]);
      calibPartNumber += (char)MODBUS_GET_LOW_BYTE(tab_reg[i]);
  }
  m_calibrationData.calibPartNumberNumber = calibPartNumber;

#if DEBUG
  std::cout << "Part Number is " << m_calibrationData.calibPartNumberNumber << std::endl;
#endif
  std::string calibFamilyID;
  for (unsigned int i = 20; i < 22; i++)
  {
      calibFamilyID += (char)MODBUS_GET_HIGH_BYTE(tab_reg[i]);
      calibFamilyID += (char)MODBUS_GET_LOW_BYTE(tab_reg[i]);
  }
  m_calibrationData.calibFamilyID = calibFamilyID;

#if DEBUG
  std::cout << "Calib Family ID is " << m_calibrationData.calibFamilyID << std::endl;
#endif
  std::string calibTime;
  for (unsigned int i = 22; i < 32; i++)
  {
      calibTime += (char)MODBUS_GET_HIGH_BYTE(tab_reg[i]);
      calibTime += (char)MODBUS_GET_LOW_BYTE(tab_reg[i]);
  }

  m_calibrationData.calibTime = calibTime;
#if DEBUG
  std::cout << "Calib Time is " << m_calibrationData.calibTime << std::endl;
#endif
  std::cout << "Calibration Matrix:" << std::endl;
  for (unsigned int i = 0; i < 6; i++)
  {
    for (unsigned int j = 0; j < 6; j++)
    {
    	unsigned int index = 32+(i*6+j)*2;
        uint32_t temp;
        temp = (MODBUS_GET_HIGH_BYTE(tab_reg[index]) << 24) | (MODBUS_GET_LOW_BYTE(tab_reg[index]) << 16)  | (MODBUS_GET_HIGH_BYTE(tab_reg[index+1]) << 8) | (MODBUS_GET_LOW_BYTE(tab_reg[index+1]));
        memcpy(&m_calibrationData.basicMatrix[i][j], &temp, sizeof(float));
        std::cout << m_calibrationData.basicMatrix[i][j] << ", ";
    }
    std::cout << std::endl;
  }

  m_calibrationData.forceUnitsInt = static_cast<ForceUnit>(MODBUS_GET_HIGH_BYTE(tab_reg[104]));
#if DEBUG
  std::cout << "Force Units are ";
  switch (m_calibrationData.forceUnitsInt)
  {
  case ForceUnit::FU_POUNDS:
	  std::cout << "Pound" << std::endl;
	  break;
  case ForceUnit::FU_NEWTON:
	  std::cout << "Newton" << std::endl;
	  break;
  case ForceUnit::FU_KILOPOUND:
	  std::cout << "Kilopound" << std::endl;
	  break;
  case ForceUnit::FU_KILONEWTON:
	  std::cout << "KiloNewton" << std::endl;
	  break;
  case ForceUnit::FU_K_EQV_FORCE:
	  std::cout << "Kilogram-equivalent force" << std::endl;
	  break;
  case ForceUnit::FU_G_EQV_FORCE:
	  std::cout << "Gram-equivalent force" << std::endl;
	  break;
  default:
	  std::cout << "Unknown!" << std::endl;
	  break;
  }
#endif
  forceUnitStr = ForceUnitNames[m_calibrationData.forceUnitsInt-1];

  m_calibrationData.torqueUnitsInt = static_cast<TorqueUnit>(MODBUS_GET_LOW_BYTE(tab_reg[104]));
#if DEBUG
  std::cout << "Torque Units are ";
  switch (m_calibrationData.torqueUnitsInt)
  {
  case TorqueUnit::TU_POUNDS_INCH:
	  std::cout << "Pound-inch" << std::endl;
	  break;
  case TorqueUnit::TU_POUNDS_FOOT:
	  std::cout << "Pound-foot" << std::endl;
	  break;
  case TorqueUnit::TU_NEWTON_METER:
	  std::cout << "Newton-meter" << std::endl;
	  break;
  case TorqueUnit::TU_NEWTON_MILITMETER:
	  std::cout << "Newton-millimeter" << std::endl;
	  break;
  case TorqueUnit::TU_K_EQV_CENTIMETER:
	  std::cout << "Kilogram(-equivalent)-centimeter" << std::endl;
	  break;
  default:
	  std::cout << "Unknown!" << std::endl;
	  break;
  }
#endif
  torqueUnitStr = TorqueUnitNames[m_calibrationData.torqueUnitsInt-1];

  for (unsigned int i = 0; i < 6; i++)
  {
	  unsigned int index = 105 +i*2;
      int32_t temp;
      temp = (((int32_t)tab_reg[index]) << 16) | tab_reg[index+1];
      memcpy(&m_calibrationData.maxRating[i], &temp, sizeof(float));
  }
  m_calibrationData.countsPerForce =  MODBUS_GET_INT32_FROM_INT16(tab_reg, 117);
#if DEBUG
  std::cout << "Counts per force are " << m_calibrationData.countsPerForce << std::endl;
#endif
  m_calibrationData.countsPerTorque = MODBUS_GET_INT32_FROM_INT16(tab_reg, 119);
#if DEBUG
  std::cout << "Counts per torque " << m_calibrationData.countsPerTorque << std::endl;
#endif
  for (unsigned int i = 0; i < 6; i++)
  {
  	m_calibrationData.gageGain[i] = (MODBUS_GET_HIGH_BYTE(tab_reg[121 + i]) << 8) | (MODBUS_GET_LOW_BYTE(tab_reg[121 + i]));
#if DEBUG
  	std::cout << "Gage gain " << i << " is " << m_calibrationData.gageGain[i] << std::endl;
#endif
  }
  for (unsigned int i = 0; i < 6; i++)
  {
  	m_calibrationData.gageOffset[i] = (MODBUS_GET_HIGH_BYTE(tab_reg[127 + i]) << 8) | (MODBUS_GET_LOW_BYTE(tab_reg[127 + i]));
#if DEBUG
  	std::cout << "Gage offset " << i << " is " << m_calibrationData.gageOffset[i] << std::endl;
#endif
  }

  return true;
}

bool ATIForceTorqueSensorHWRS485::SetStorageLock(const bool lock) const
{
  uint8_t lockCode;
  if (lock)
  {
	lockCode = 0x18;
  }
  else
  {
	lockCode = 0xaa;
  }

  uint8_t raw_req[] = { m_ModbusBaseIdentifier, 0x6a, lockCode };		//OpCode is 0x6a = 104, Data is 0x18 (to lock) or 0xaa (to unlock) as specified in 8.3.1.
  uint8_t rsp[MODBUS_RTU_MAX_ADU_LENGTH];
  int req_length = modbus_send_raw_request(m_modbusCtrl, raw_req, 3 * sizeof(uint8_t));
  int len = modbus_receive_confirmation(m_modbusCtrl, rsp);

  if (len == -1)
  {
    std::cout << "Could not set storage lock. No confirmation received." << std::endl;
    return false;
  }
  if (rsp[2] != 1)
  {
    std::cout << "Could not set storage lock. Invalid answer." << std::endl;
    return false;
  }
#if DEBUG
  std::cout << "Setting storage lock was successful" << std::endl;
#endif

  return true;
}


bool ATIForceTorqueSensorHWRS485::SetActiveGain(const uint16_t ag0, const uint16_t ag1, const uint16_t ag2, const uint16_t ag3, const uint16_t ag4, const uint16_t ag5) const
{
#if DEBUG
	std::cout << "Setting active gain to " << ag0 << ", " <<  ag1 << ", " << ag2 << ", " << ag3 << ", " << ag4 << ", " << ag5 << std::endl;
#endif
	uint16_t activeGain[6] = {ag0, ag1, ag2, ag3, ag4, ag5};
	int rc = modbus_write_registers(m_modbusCtrl, 0x0000, 6, activeGain);
	if (rc == -1)
	{
		std::cout << "Setting active gain failed with status " << rc << std::endl;
		fprintf(stderr, "%s\n", modbus_strerror(errno));
		return false;
	}
	return true;
}


bool ATIForceTorqueSensorHWRS485::SetActiveOffset(const uint16_t ao0, const uint16_t ao1, const uint16_t ao2, const uint16_t ao3, const uint16_t ao4, const uint16_t ao5) const
{
#if DEBUG
	std::cout << "Setting active offset to " << ao0 << ", " <<  ao1 << ", " << ao2 << ", " << ao3 << ", " << ao4 << ", " << ao5 << std::endl;
#endif
	uint16_t activeOffset[6] = {ao0, ao1, ao2, ao3, ao4, ao5};
	int rc = modbus_write_registers(m_modbusCtrl, 0x0006, 6, activeOffset);
	if (rc == -1)
	{
		std::cout << "Setting active offset failed with status " << rc << std::endl;
		fprintf(stderr, "%s\n", modbus_strerror(errno));
		return false;
	}
	return true;
}

bool ATIForceTorqueSensorHWRS485::ReadStatusWord() const
{
#if DEBUG
  std::cout << "\n\n*********FTStatusWord**********" << std::endl;
#endif
  uint16_t tab_reg[1];

  int rc = modbus_read_registers(m_modbusCtrl, 0x001D, 1, tab_reg);
  if (rc == -1)
  {
	  std::cout << "Reading Status Word failed with status " << rc << std::endl;
      fprintf(stderr, "%s\n", modbus_strerror(errno));
      return false;
  }
  bool statusGood = true;
  if (tab_reg[0] == 0)
  {
#if DEBUG
	  std::cout << "Status is good." << std::endl;
#endif
      return true;
  }
  
  if (tab_reg[0] & ST_WATCHDOG_RESET)
  {
	  std::cout << "Watchdog reset - the analog board was reset by the watchdog timer." << std::endl;
  }
  if (tab_reg[0] & ST_EXC_VOLTAGE_HIGH)
  {
	  std::cout << "Excitation voltage too high." << std::endl;
  }
  if (tab_reg[0] & ST_EXC_VOLTAGE_LOW)
  {
	  std::cout << "Excitation voltage too low." << std::endl;
  }
  if (tab_reg[0] & ST_ART_ANALOG_GRND_OOR)
  {
	  std::cout << "Artificial analog ground out of range." << std::endl;
  }
  if (tab_reg[0] & ST_PWR_HIGH)
  {
	  std::cout << "Power supply too high." << std::endl;
  }
  if (tab_reg[0] & ST_PWR_LOW)
  {
	  std::cout << "Power supply too low." << std::endl;
  }
  if (tab_reg[0] & ST_EEPROM_ERR)
  {
	  std::cout << "Error accessing stored settings in EEPROM." << std::endl;
  }
  if (tab_reg[0] & ST_INV_CONF_DATA)
  {
	  std::cout << "Invalid configuration data." << std::endl;
  }
  if (tab_reg[0] & ST_STRAIN_GAGE_SUPPLY_HIGH)
  {
	  std::cout << "Strain gage bridge supply current too high." << std::endl;
  }
  if (tab_reg[0] & ST_STRAIN_GAGE_SUPPLY_LOW)
  {
	  std::cout << "Strain gage bridge supply current too low." << std::endl;
  }
  if (tab_reg[0] & ST_THERMISTOR_HIGH)
  {
	  std::cout << "Thermistor too high." << std::endl;
  }
  if (tab_reg[0] & ST_THERMISTOR_LOW)
  {
	  std::cout << "Thermistor too low." << std::endl;
  }
  if (tab_reg[0] & ST_DAC_READING_OOR)
  {
	  std::cout << "DAC reading out of range." << std::endl;
  }
  return false;
}

bool ATIForceTorqueSensorHWRS485::SetBaudRate(const int value, const bool setVolatile)
{
#if DEBUG
  std::cout << "\n\n*******Setting Baud Rate value to: " << value << "********" << std::endl;
#endif
  uint16_t baudrateIndex = 0;
  if (value == MODBUSBAUD_1250K)
  {
	  baudrateIndex = 0;
  }
  else if (value == MODBUSBAUD_115200)
  {
	  baudrateIndex = 2;
  }
  else if (value == MODBUSBAUD_19200)
  {
	  baudrateIndex = 1;
  }
  else //Baudrate not supported
  {
	  fprintf(stderr, "Baudrate %i is not supported \n", value);
	  return false;
  }

  //If Baudrate should be set non-volatile, the 0x001E flag has to be set first (According to 8.3.2 of the sensors user manual)
  if(!setVolatile)
  {
	  uint16_t tab_reg[1] = {1};

	  int rc = modbus_write_registers(m_modbusCtrl, 0x001E, 1, tab_reg);
	  if (rc == -1)
	  {
		  std::cout << "Setting non-volatile baud rate flag failed with status " << rc << std::endl;
	      fprintf(stderr, "%s\n", modbus_strerror(errno));
	      return false;
	  }
  }

  uint16_t tab_reg[1] = {baudrateIndex};

  int rc = modbus_write_registers(m_modbusCtrl, 0x001F, 1, tab_reg);
  if (rc == -1)
  {
	  std::cout << "Setting baudrate failed with status " << rc << std::endl;
      fprintf(stderr, "%s\n", modbus_strerror(errno));
      return false;
  }
  m_ModbusBaudrate = value;
  //Reset connection here
  Reset();

  return true;
}

bool ATIForceTorqueSensorHWRS485::Reset()
{
#if DEBUG
  std::cout << "\n\n******* Reseting the RS485 Interface ********" << std::endl;
#endif
  if (!Close())
  {
	  return false;
  }
  if (!init())
  {
	  return false;
  }
  return true;
}

bool ATIForceTorqueSensorHWRS485::Close()
{
	modbus_close(m_modbusCtrl);
	modbus_free(m_modbusCtrl);

	return true;
}

bool ATIForceTorqueSensorHWRS485::SetSessionID(const uint16_t sessionID)
{
#if DEBUG
  std::cout << "\n\n*******Setting Session ID value to HEX : " << std::hex << sessionID << " ********" << std::endl;
#endif
  uint16_t tab_reg[1] = {sessionID};

  int rc = modbus_write_registers(m_modbusCtrl, 0x000C, 1, tab_reg);
  if (rc == -1)
  {
	  #if DEBUG
		  std::cout << "Setting Session ID failed with status " << rc << std::endl;
	  #endif
      fprintf(stderr, "%s\n", modbus_strerror(errno));
      return false;
  }
  return true;
}

bool ATIForceTorqueSensorHWRS485::ReadSessionID(uint16_t &sessionID) const
{
#if DEBUG
  std::cout << "\n\n*******Reading Session ID"<< " ********"
            << std::endl;
#endif
  uint16_t tab_reg[1];

  int rc = modbus_read_registers(m_modbusCtrl, 0x000C, 1, tab_reg);
  if (rc == -1)
  {
	  #if DEBUG
		  std::cout << "Reading Session ID failed with status " << rc << std::endl;
	  #endif
      fprintf(stderr, "%s\n", modbus_strerror(errno));
      return false;
  }
  sessionID = tab_reg[0];
  return true;
}


bool ATIForceTorqueSensorHWRS485::readFTData(int statusCode, double& Fx, double& Fy, double& Fz, double& Tx, double& Ty, double& Tz)
{
	int sg0 = 0, sg1 = 0, sg2 = 0, sg3 = 0, sg4 = 0, sg5 = 0;
	if (!m_isStreaming)
	{
		if (!StartStreaming())
		{
			return false;
		}
	}

	m_buffer_mutex.lock();
	long unsigned int timeSinceLastInput = (ros::Time::now()-m_buffer.timestamp).toNSec();
	if (timeSinceLastInput > m_bufferTimeout)
	{
		std::cout << "Buffer timestamp is too far in the past! Data might be deprecated and will be ignored. (" << timeSinceLastInput << ")" << std::endl;
		m_buffer_mutex.unlock();
		return false;
	}
	sg0 = m_buffer.gageData[0];
	sg1 = m_buffer.gageData[1];
	sg2 = m_buffer.gageData[2];
	sg3 = m_buffer.gageData[3];
	sg4 = m_buffer.gageData[4];
	sg5 = m_buffer.gageData[5];
	m_buffer_mutex.unlock();

	StrainGaugeToForce(sg0, sg1, sg2, sg3, sg4, sg5);
	Fx = m_vForceData(0);
	Fy = m_vForceData(1);
	Fz = m_vForceData(2);
	Tx = m_vForceData(3);
	Ty = m_vForceData(4);
	Tz = m_vForceData(5);
//	std::cout << "Strain Gauge is: " << sg0 << ", " << sg1 << ", " << sg2 << ", " << sg3 << ", " << sg4 << ", " << sg5 << ", Force data is: " << Fx << " " << forceUnitStr << ", " << Fy << " " << forceUnitStr << ", "
//			<< Fz << " " << forceUnitStr << ", Torque data is: " <<  Tx << " " << torqueUnitStr << ", " << Ty << " " << torqueUnitStr << ", " << Tz << " " << torqueUnitStr << std::endl;

	return true;

}

bool ATIForceTorqueSensorHWRS485::StartStreaming()
{
#if DEBUG
	std::cout << "Trying to start stream" << std::endl;
#endif
	if (!m_isStreaming)
	{
		uint8_t raw_req[] = { m_ModbusBaseIdentifier, 0x46, 0x55 };		//OpCode is 0x46 = 70, Data is 0x55 as specified in 8.3.1.
		uint8_t rsp[MODBUS_RTU_MAX_ADU_LENGTH];
		int req_length = modbus_send_raw_request(m_modbusCtrl, raw_req, 3 * sizeof(uint8_t));
		int len = modbus_receive_confirmation(m_modbusCtrl, rsp);

		if (len == -1)
		{
			std::cout << "Could not start data streaming. No confirmation received." << std::endl;
			return false;
		}
		else if (rsp[2] == 1)
		{
			if (!OpenRawConnection())
			{
				return false;
			}
			m_isStreaming = true;
		}
		else
        {
            std::cout << "Could not start data streaming. Invalid answer." << std::endl;
            return false;
        }
		bytesRead = 0;
		std::cout << "Started streaming." << std::endl;
		lastValidTimeStamp = ros::Time::now();
		bufferSize = 0;
		m_readThread = boost::thread(&ATIForceTorqueSensorHWRS485::ReadDataLoop, this);
		m_readThread.detach();
	}
	return true;
}

bool ATIForceTorqueSensorHWRS485::OpenRawConnection()
{
	//Confirmed
	/* Open File Descriptor: The connection have read/write access and it should be blocking */
	m_rs485 = open( m_RS485Device.c_str(),   O_RDWR | O_NOCTTY |  O_EXCL );

	/* Error Handling */
	if ( m_rs485 == 0 )
	{
		std::cout << "Error " << errno << " opening " << m_RS485Device << ": " << strerror (errno) << std::endl;
		return false;
	}
	//Flush serial buffer
	tcflush(m_rs485,TCIOFLUSH);

	struct serial_struct ss;
    ioctl(m_rs485, TIOCGSERIAL, &ss);
    ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
    ss.custom_divisor = (ss.baud_base + (m_ModbusBaudrate/ 2)) / m_ModbusBaudrate;
    int closestSpeed = ss.baud_base / ss.custom_divisor;

    if (closestSpeed < m_ModbusBaudrate * 98 / 100 || closestSpeed > m_ModbusBaudrate * 102 / 100) {
        fprintf(stderr, "Cannot set serial port speed to. Closest possible is %i\n", closestSpeed);
        return false;
    }
    ioctl(m_rs485, TIOCSSERIAL, &ss);
    struct termios tios;
    memset(&tios, 0, sizeof(struct termios));
    speed_t speed;
    speed = B38400;
    cfsetispeed(&tios, B38400);
    cfsetospeed(&tios, B38400);
    /* Set the baud rate */
    if ((cfsetispeed(&tios, speed) < 0) ||
        (cfsetospeed(&tios, speed) < 0)) {
        close(m_rs485);
        m_rs485 = 0;
        std::cout << "Could not set speed" << std::endl;
        return false;

    }
    ioctl(m_rs485, TIOCSSERIAL, &ss);
    /* Software flow control is disabled */
    tios.c_iflag &= ~(IXON | IXOFF | IXANY);
    /* Raw ouput */
    tios.c_oflag &=~ OPOST;
    tios.c_cflag &= ~CSIZE;
    tios.c_cflag |= CS8;
    tios.c_cflag &=~ CSTOPB;
    tios.c_cflag |= PARENB;
    tios.c_cflag &=~ PARODD;
    tios.c_iflag |= INPCK;
    tios.c_cflag |= (CREAD | CLOCAL);	//enable receiver
    tios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tios.c_cc[VMIN] = 39;		/* Read 10 characters */
    tios.c_cc[VTIME] = 0;		/* Wait indefinitely   */
    if (tcsetattr(m_rs485, TCSANOW, &tios) < 0)
    {
        return false;
    }
    return true;
}

void ATIForceTorqueSensorHWRS485::ReadDataLoop()
{
	while (m_isStreaming)
	{
		if (ReadData())
		{
			lastValidTimeStamp = ros::Time::now();
		}
		else if ((ros::Time::now() - lastValidTimeStamp).toSec() > 2)
		{
			std::cout << "Error while reading: Timeout. " << std::endl;
			SendStopSequence();
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			ReadStatusWord();
			m_isStreaming = false;
			return;
		}
	}
}

bool ATIForceTorqueSensorHWRS485::ReadData()
{
	if (m_isStreaming)
	{
#if DEBUG
		readStart = ros::Time::now();
#endif
		/* *** READ *** */
		int n = read( m_rs485, &streamBuf + bufferSize , sizeof(streamBuf) - bufferSize );
		/* Error Handling */
		if (n < 0)
		{
			std::cout << "Error reading: " << strerror(errno) << std::endl;
			std::chrono::microseconds sleepTime = std::chrono::microseconds(5000);
			return false;
		}
		else if (n == 0) {
			//No data received
			std::cout << "No data received!" << std::endl;
			std::chrono::microseconds sleepTime = std::chrono::microseconds(5000);
			return false;
		}
		else if (n + bufferSize != sizeof(streamBuf)) {
			//The received package was incomplete, therefore we need to read the missing data in the next call
			std::cout << "Data truncated. n: " << n << " bufferSize: " << bufferSize << std::endl;
			bufferSize = (n + bufferSize) % sizeof(streamBuf);
			bytesRead += n;
			return false;
		}
		bytesRead += sizeof(streamBuf);
		bufferSize = 0;
		//Validate the read data (if invalid, exit)
		if (!ValidateFTData(streamBuf))
		{
			return false;
		}
		GageVector tmp;
		//The data package from the sensor contains the data clustered together by each axis (as specified in 8.4.2)
		//Therefore, the data has to be rearranged when saving it
		tmp.gageData[0] = streamBuf[1] | (streamBuf[0] << 8);
		tmp.gageData[1]  = streamBuf[7] | (streamBuf[6] << 8);
		tmp.gageData[2]  = streamBuf[3] | (streamBuf[2] << 8);
		tmp.gageData[3]  = streamBuf[9] | (streamBuf[8] << 8);
		tmp.gageData[4]  = streamBuf[5] | (streamBuf[4] << 8);
		tmp.gageData[5]  = streamBuf[11] | (streamBuf[10] << 8);
		tmp.timestamp = ros::Time::now();
		if (m_buffer_mutex.try_lock())
		{
			m_buffer = tmp;
			m_buffer_mutex.unlock();
		}
#if DEBUG
		std::cout << "Read took " << (ros::Time::now()-readStart).toNSec() << " nanoseconds" << std::endl;
#endif

	}
	return true;
}

bool ATIForceTorqueSensorHWRS485::ValidateFTData(const uint8_t (&buf)[26]) const
{
	uint8_t check = buf[12];

	if (((check >> 7) & 0x01))	//Check status bit
	{
		//If it is set to 1, an error occurred -> Stop streaming
		std::cout << bytesRead << "Invalid status bit" << std::endl;
		return false;
	}
	uint8_t checksum = check & 0x7F;

	int compareChecksum = 0;
	for (unsigned int i = 0; i < 12; i++)
	{
		compareChecksum += buf[i];
	}
	if ((compareChecksum & 0x7F) != checksum)
	{
		std::cout << bytesRead << "Package has invalid checksum! Ignoring data... ";
		return false;
	}
	return true;
}


bool ATIForceTorqueSensorHWRS485::StopStreaming()
{
	if (m_isStreaming)
	{
		if (m_readThread.joinable())
		{
			m_readThread.join();
		}
		m_isStreaming = false;
		return SendStopSequence();
	}
	return false;
}

bool ATIForceTorqueSensorHWRS485::SendStopSequence()
{
	if (m_isStreaming)
	{
		unsigned char stopCmd[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	//Send jamming sequence of 14 bytes to stop streaming
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		int n_written = write( m_rs485, stopCmd, sizeof(stopCmd));
		return true;
	}
	return false;
}

void ATIForceTorqueSensorHWRS485::StrainGaugeToForce(const int &sg0, const int &sg1, const int &sg2, const int &sg3, const int &sg4, const int &sg5)
{
  Eigen::VectorXf v6SG(6);
  Eigen::VectorXf tmp(6);

  v6SG[0] = sg0;
  v6SG[1] = sg1;
  v6SG[2] = sg2;
  v6SG[3] = sg3;
  v6SG[4] = sg4;
  v6SG[5] = sg5;
  tmp = m_mXCalibMatrix * v6SG;
  //Scale the force/torque values according to calibration
  tmp[0] = tmp[0] / m_calibrationData.countsPerForce;
  tmp[1] = tmp[1] / m_calibrationData.countsPerForce;
  tmp[2] = tmp[2] / m_calibrationData.countsPerForce;
  tmp[3] = tmp[3] / m_calibrationData.countsPerTorque;
  tmp[4] = tmp[4] / m_calibrationData.countsPerTorque;
  tmp[5] = tmp[5] / m_calibrationData.countsPerTorque;
  m_vForceData = tmp;
}

void ATIForceTorqueSensorHWRS485::SetCalibMatrix(const float (&matrix)[6][6])
{
  Eigen::MatrixXf tmp(6, 6);
  for (unsigned int i = 0; i < 6; i++)
  {
    for (unsigned int j = 0; j < 6; j++)
    {
    	tmp(i,j) = matrix[i][j];
    }
  }

  m_mXCalibMatrix = tmp;
}

PLUGINLIB_EXPORT_CLASS(ATIForceTorqueSensorHWRS485, hardware_interface::ForceTorqueSensorHW)
