#ifndef FORCETORQUECTRL_INCLUDEDEF_H
#define FORCETORQUECTRL_INCLUDEDEF_H

#include <iostream>


// Headers provided by other cob-packages
#include <cob_generic_can/CanItf.h>

#include <cob_forcetorque/Mutex.h>

#include <Eigen/Core>
#include <fstream>

//opCodes for the ForceTorque Can Interface
//set as Can Message ID
#define READ_SG 	0x0
#define READ_MATRIX 	0x2
#define READ_SERIALNR	0x5
#define SET_CALIB	0x6
#define READ_COUNTSPERU	0x7
#define READ_UNITCODE	0x8
#define READ_DIAGNOV	0X9
#define RESET		0xC
#define SET_BASEID	0xD
#define SET_BAUD	0xE
#define READ_FIRMWARE	0xF

#define ATI_CAN_BAUD_2M		0
#define ATI_CAN_BAUD_1M		1
#define ATI_CAN_BAUD_500K	3
#define ATI_CAN_BAUD_250K	7

class ForceTorqueCtrl
{
    public:
	ForceTorqueCtrl();
	ForceTorqueCtrl(int can_type, std::string can_path, int can_baudrate, int base_identifier);
	~ForceTorqueCtrl();

	bool Init();
	bool ReadFTSerialNumber();
	bool ReadCountsPerUnit();
	bool ReadUnitCodes();
	bool SetActiveCalibrationMatrix(int num);
	bool SetBaudRate(int value);
	bool SetBaseIdentifier(int identifier);
	bool Reset();
	void ReadSGData(int statusCode, double &Fx, double &Fy, double &Fz, double &Tx, double &Ty, double &Tz);
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
	Mutex m_Mutex;
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
