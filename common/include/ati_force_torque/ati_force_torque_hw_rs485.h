/****************************************************************
 *
 * Copyright 2016 Intelligent Industrial Robotics (IIROB) Group,
 * Institute for Anthropomatics and Robotics (IAR) -
 * Intelligent Process Control and Robotics (IPR),
 * Karlsruhe Institute of Technology
 *
 * Maintainer: Denis ������������������togl, email: denis.stogl@kit.edu
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * Date of creation: May 2018
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

#ifndef ATIForceTorqueSensorHWRS485_INCLUDEDEF_H
#define ATIForceTorqueSensorHWRS485_INCLUDEDEF_H
// Communication Class for the ATI FT sensor over a FS485 connecting using the modbus protocol
// If not specified otherwise, all documentation references in this class refer to:
// https://www.ati-ia.com/app_content/documents/9620-05-Digital%20FT.pdf

#include <iostream>
#include <fstream>
#include <Eigen/Core>

#include <libmodbus/modbus-rtu.h>
#include <ros/time.h>
#include <ros/ros.h>
#include <mutex>
#include <thread>
#include <boost/thread.hpp>

#include <force_torque_sensor/force_torque_sensor_hw.h>

//Available Modbus Baudrates (As specified in 8.1):
#define MODBUSBAUD_1250K		1250000
#define MODBUSBAUD_115200	115200
#define MODBUSBAUD_19200	19200

enum ForceUnit
{
	FU_POUNDS = 1,
	FU_NEWTON,
	FU_KILOPOUND,
	FU_KILONEWTON,
	FU_K_EQV_FORCE,
	FU_G_EQV_FORCE
};

enum TorqueUnit
{
	TU_POUNDS_INCH = 1,
	TU_POUNDS_FOOT,
	TU_NEWTON_METER,
	TU_NEWTON_MILITMETER,
	TU_K_EQV_CENTIMETER,
	TU_KILONEWTONMETER
};

const std::string ForceUnitNames[] = {"lb", "N", "kip", "KN", "kp", "gf"};
const std::string TorqueUnitNames[] = {"lb-in", "lb-ft", "Nm", "Nmm", "", "KNm"};

//Available status words (As specified in 8.4.3):
#define ST_WATCHDOG_RESET 1
#define ST_EXC_VOLTAGE_HIGH 2
#define ST_EXC_VOLTAGE_LOW 4
#define ST_ART_ANALOG_GRND_OOR 8
#define ST_PWR_HIGH 16
#define ST_PWR_LOW 32
//-------------------------Bit 6 not used
#define ST_EEPROM_ERR 128
#define ST_INV_CONF_DATA 256
#define ST_STRAIN_GAGE_SUPPLY_HIGH 512
#define ST_STRAIN_GAGE_SUPPLY_LOW 1024
#define ST_THERMISTOR_HIGH 2048
#define ST_THERMISTOR_LOW 4096
#define ST_DAC_READING_OOR 8192

//Encapsulates all calibration data available from the sensor. Mirrors the data structure specified in 8.4.4.
struct CalibrationData
{
	  std::string calibSerialNumber;
	  std::string calibPartNumberNumber;
	  std::string calibFamilyID;
	  std::string calibTime;
	  float basicMatrix[6][6];
	  ForceUnit forceUnitsInt;
	  TorqueUnit torqueUnitsInt;
	  float maxRating[6];
	  int32_t countsPerForce;
	  int32_t countsPerTorque;
	  uint16_t gageGain[6];
	  uint16_t gageOffset[6];
};

//A gage vector containing the ft data from a single sensor response.
struct GageVector
{
	int16_t gageData[6];
	ros::Time timestamp;
};

class ATIForceTorqueSensorHWRS485 : public hardware_interface::ForceTorqueSensorHW
{
public:
	/**

	*/
	ATIForceTorqueSensorHWRS485();

	/**

		@param device_path
		@param device_baudrate
		@param base_identifier
	*/
	ATIForceTorqueSensorHWRS485(std::string device_path, int device_baudrate, int base_identifier);

	/**
		Standard destructor. Closes all active connections
	*/
	virtual ~ATIForceTorqueSensorHWRS485();

	/**
	 * Initializes communication with the sensor using the current configuration, reads calibration data and sets the calibration matrix
	 * Use initCommunication(...) prior to this, in order to set the communication parameters
	    @return True, if the initialization was successful
	*/
	virtual bool init();

	/**
		Sets the parameters used for Modbus communication
		@param type	Not used in this implementation
		@param path Device path on the system
		@param baudrate Baudrate, with which to communicate. Will only accept 1250000, 115200 or 19200
		@param base_identifier Sensor ID. Default value is 0xA
	    @return True, if all parameters were valid
	*/
	virtual bool initCommunication(int type, std::string path, int baudrate, int base_identifier);

	/**
	    Reads the calibration data from the FT sensor and stores it

		@param calibrationNumber: The register number of the calibration data to read. Can be set to a value between 1 and 16, according to 8.3.2
	    @return Returns true, if the operation was successful
	*/
	bool ReadFTCalibrationData(const unsigned int calibrationNumber = 1);

	/**
		Reads the current status from the sensor
		This should be done after an error occurred, to determine the exact cause of the problem
		The result will then be printed to the console
	    @return Returns true if the status word could be read successfully
	*/
	bool ReadStatusWord() const;

	/**
		Set the communication baudrate of the sensor
		If the baudrate change was successful, resets the communication and creates a new connection using the given baudrate
		@param value The baurate (in baud/sec). Allowed values are: 1250000, 115200, 19200
		@param setVolatile if this flag is set, the baudrate will only be changed for this current session. If not, the sensor's baudrate will be changed permanently
	    @return Returns true if the operation was successful
	*/
	bool SetBaudRate(const int value, const bool setVolatile = true);

	/**
		@param sessionID
	    @return
	*/
	bool SetSessionID(const uint16_t sessionID);

	/**

		@param sessionID
	    @return
	*/
	bool ReadSessionID(uint16_t &sessionID) const;

	/**
		Closes the current connection (if open) and opens a new connection
	    @return Returns true if the operation was successful
	*/
	bool Reset();

	/**
		Closes the current modbus communication with the sensor
	    @return Returns true if the operation was successful
	*/
	bool Close();

	/**
	    @return
	*/
	bool StartStreaming();

	/**
	    @return
	*/
	bool StopStreaming();

	/**
		@param statusCode
		@param Fx
		@param Fy
		@param Fz
		@param Tx
		@param Ty
		@param Tz
	    @return
	*/
	virtual bool readFTData(int statusCode, double& Fx, double& Fy, double& Fz, double& Tx, double& Ty, double& Tz);

	/** Transmits the active gain values to the sensor. You need to unlock the storage lock first before being able to set the values
		@param ag0
		@param ag1
		@param ag2
		@param ag3
		@param ag4
		@param ag5
	    @return
	*/
	bool SetActiveGain(const uint16_t ag0, const uint16_t ag1, const uint16_t ag2, const uint16_t ag3, const uint16_t ag4, const uint16_t ag5) const;

	/** Transmits the active offset values to the sensor. You need to unlock the storage lock first before being able to set the values
		@param ao0
		@param ao1
		@param ao2
		@param ao3
		@param ao4
		@param ao5
	    @return
	*/
	bool SetActiveOffset(const uint16_t ao0, const uint16_t ao1, const uint16_t ao2, const uint16_t ao3, const uint16_t ao4, const uint16_t ao5) const;

	/**
		Sets the calibration matrix
		@param matrix the new calibration matrix as a floating point array
	*/
	void SetCalibMatrix(const float (&matrix)[6][6]);

	/**

		@param sg0:
		@param sg1:
		@param sg2:
		@param sg3:
		@param sg4:
		@param sg5:
	    @return
	*/
	void StrainGaugeToForce(const int &sg0, const int &sg1, const int &sg2, const int &sg3, const int &sg4, const int &sg5);
protected:
	/**
		Openes modbus connection to the sensor
	    @return Returns true, if the operation was successful
	*/
	bool initRS485();

	/**
	    Sets the storage mode of the connected sensor (according to specifications in 8.3.1. of the sensor's use manual)

	    @param lock: True, to lock the sensor's storage, false to unlock it
	    @return Returns true, if the operation was successful
	*/
	bool SetStorageLock(const bool lock) const;

	/**
	    Reads a set of force torque measurements in the form of a GageVector from the connected sensor and stores it in the local m_buffer variable

	    @return Returns true, if the operation was successful. Returns false if the checksum was invalid or no data was received
	*/
	bool ReadData();
	void ReadDataLoop();

	bool ValidateFTData(const uint8_t (&buf)[26]) const;

	bool SendStopSequence();

private:
	modbus_t* m_modbusCtrl;							//Modbus handle to the rs485 port
	int m_rs485;									//Raw handle to the rs485 port

	std::string m_RS485Device;						//Device path of the rs485 device
	uint8_t m_ModbusBaseIdentifier;					//ID number of the sensor, is 0xA by default
	int m_ModbusBaudrate;							//Sensor baudrate, must be set to 1250000, 115200 or 19200

	Eigen::MatrixXf m_mXCalibMatrix;				//6-D Matrix Calibration Matrix, transforms the Raw Gage data into FT values
	Eigen::MatrixXf m_vForceData;					//6-D Vector: Latest Force-Torque values from the sensor

	bool m_isStreaming = false;						//Indicates whether the sensor is streaming ft data and the stream reader thread is running

	long unsigned int m_bufferTimeout = 10000000;
	long unsigned int bytesRead;

	CalibrationData m_calibrationData;				//Calibration data object from the sensor. Can be set using ReadFTCalibrationData
	ros::Time lastValidTimeStamp;					//The timestamp of the last valid data package
	uint8_t streamBuf [26];							//Stream buffer. Contains the latest streaming data from the sensor
	ros::Time readStart;
	unsigned int bufferSize = 0;					//Contains the size of the current buffer

	GageVector m_buffer;				//Contains the latest reading from the sensor
	std::mutex m_buffer_mutex;			//Mutex used to prevent external data requests from interfering with the reader thread
	boost::thread m_readThread;			//This thread runs in the background and keeps updating the buffer by reading new data from the sensor

	std::string forceUnitStr;			//The current force unit's name
	std::string torqueUnitStr;			//The current torque unit's name

};

#endif /* ATIForceTorqueSensorHWRS485_INCLUDEDEF_H */
