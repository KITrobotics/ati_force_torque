//general includes
#include <unistd.h>

// Headrs provided by cob-packages
//#include <cob_generic_can/CanESD.h>
//#include <cob_generic_can/CanPeakSys.h>
#include <cob_generic_can/CanPeakSysUSB.h>
#include <cob_forcetorque/ForceTorqueCtrl.h>

ForceTorqueCtrl::ForceTorqueCtrl()
{
	// ------------- first of all set used CanItf
	m_pCanCtrl = NULL;

	// for types and baudrates see: https://github.com/ipa320/cob_robots/blob/hydro_dev/cob_hardware_config/raw3-5/config/base/CanCtrl.ini
	m_CanType = CANITFTYPE_CAN_PEAK_USB;
	m_CanDevice = "/dev/pcan32";
	m_CanBaudrate = CANITFBAUD_250K;
	m_CanBaseIdentifier = 0x20 << 4;
}

ForceTorqueCtrl::ForceTorqueCtrl(int can_type, std::string can_path, int can_baudrate, int base_identifier)
{
	// ------------- first of all set used CanItf
	m_pCanCtrl = NULL;

	// for types and baudrates see: https://github.com/ipa320/cob_robots/blob/hydro_dev/cob_hardware_config/raw3-5/config/base/CanCtrl.ini
	m_CanType = can_type;
	m_CanDevice = can_path;
	m_CanBaudrate = can_baudrate;
	m_CanBaseIdentifier = base_identifier << 4;
}

ForceTorqueCtrl::~ForceTorqueCtrl()
{

	if (m_pCanCtrl != NULL) {
		delete m_pCanCtrl;
	}
}

bool ForceTorqueCtrl::Init()
{
	bool ret = true;

	if (initCan()) {

		// This is way of testig if communication is also successful
		if (! ReadFTSerialNumber())
		{
			std::cout << "Can not read Serial Number from FTS!" << std::endl;
			ret = false;
		}
		if (! ReadFirmwareVersion())
		{
			std::cout << "Can not read Firmware version from FTS!" << std::endl;
			ret = false;
		}
		if (! ReadCountsPerUnit())
		{
			std::cout << "Can not read Counts Per Unit from FTS!" << std::endl;
			ret = false;
		}		
		if (! ReadUnitCodes())
		{
			std::cout << "Can not read Unit Codes from FTS!" << std::endl;
			ret = false;
		}
		// Add return values and checking
		SetActiveCalibrationMatrix(0);
		ReadCalibrationMatrix();
	}
	else {
		std::cout << "CAN initialisation unsuccessful!" << std::endl;
		ret = false;
	}

	return ret;
}

bool ForceTorqueCtrl::initCan()
{
	bool ret = true;

	// current implementation only for CanPeakSysUSB
	// Should be changed to static in CanItf.h
	if (m_CanType == CANITFTYPE_CAN_PEAK_USB)
	{
		m_pCanCtrl = new CANPeakSysUSB(m_CanDevice.c_str(), m_CanBaudrate);
		std::cout << "Uses CAN-Peak-USB" << std::endl;
		ret = m_pCanCtrl->init_ret();
	}

	return ret;
}

bool ForceTorqueCtrl::ReadFTSerialNumber()
{
	std::cout << "\n\n*********FTSerialNumber**********" << std::endl;
	bool ret = true;
	CanMsg CMsg;
	CMsg.setID(m_CanBaseIdentifier | READ_SERIALNR);
	CMsg.setLength(0);

	ret = m_pCanCtrl->transmitMsg(CMsg, true);

	if (ret) {
		CanMsg replyMsg;
		replyMsg.set(0, 0, 0, 0, 0, 0, 0, 0);
		ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);

		if (ret) {
			std::cout << "reply ID: \t" << std::hex << replyMsg.getID()<<std::endl;
			std::cout << "reply Length: \t" << replyMsg.getLength()<<std::endl;
			std::cout << "reply Data: \t" << (char)replyMsg.getAt(0) << " " << (char)replyMsg.getAt(1) << " "
							  << (char)replyMsg.getAt(2) << " " << (char)replyMsg.getAt(3) << " "
							  << (char)replyMsg.getAt(4) << " " << (char)replyMsg.getAt(5) << " "
							  << (char)replyMsg.getAt(6) << " " << (char)replyMsg.getAt(7) << std::endl;
		}
		else {
			std::cout << "ForceTorqueCtrl::ReadFTSerialNumber(): Can not read message!" << std::endl;
		}
	}
	else {
		std::cout << "ForceTorqueCtrl::ReadFTSerialNumber(): Can not transmit message!" << std::endl;
	}

	return ret;
}

bool ForceTorqueCtrl::ReadCountsPerUnit()
{
	std::cout << "\n\n*********Read Counts Per Unit**********" << std::endl;
	bool ret = true;
	float countsPerForce = 0, countsPerTorque = 0;
	CanMsg CMsg;
	CMsg.setID(m_CanBaseIdentifier | READ_COUNTSPERU);
	CMsg.setLength(0);

	ret = m_pCanCtrl->transmitMsg(CMsg, true);

	if (ret) {
		CanMsg replyMsg;
		ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);

		if (ret) {
			std::cout << "reply ID: \t" << std::hex << replyMsg.getID()<<std::endl;
			std::cout << "reply Length: \t" << replyMsg.getLength()<<std::endl;
			std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " "
				      << replyMsg.getAt(2) << " " << replyMsg.getAt(3) << " "
				      << replyMsg.getAt(4) << " " << replyMsg.getAt(5) << " "
				      << replyMsg.getAt(6) << " " << replyMsg.getAt(7) << std::endl;

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
		else {
			std::cout << "ForceTorqueCtrl::ReadCountsPerUnit(): Can not read message!" << std::endl;
		}
	}
	else {
		std::cout << "ForceTorqueCtrl::ReadCountsPerUnit(): Can not transmit message!" << std::endl;
	}

	std::cout << "CountsPerforce: " << countsPerForce << "  CountsPerTorque: " << countsPerTorque << std::endl;
	return ret;
}

bool ForceTorqueCtrl::ReadUnitCodes()
{
	std::cout << "\n\n*********Read Unit Codes**********" << std::endl;
	bool ret = true;
	CanMsg CMsg;
	CMsg.setID(m_CanBaseIdentifier | READ_UNITCODE);
	CMsg.setLength(0);

	ret = m_pCanCtrl->transmitMsg(CMsg, true);

	if (ret) {
		CanMsg replyMsg;
		ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);

		if (ret) {
			std::cout << "reply ID: \t" << std::hex << replyMsg.getID()<<std::endl;
			std::cout << "reply Length: \t" << replyMsg.getLength()<<std::endl;
			std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << std::endl;
		}
		else {
			std::cout << "ForceTorqueCtrl::ReadUnitCodes(): Can not read message!" << std::endl;
		}
	}
	else {
		std::cout << "ForceTorqueCtrl::ReadUnitCodes(): Can not transmit message!" << std::endl;
	}

	return ret;
}

bool ForceTorqueCtrl::SetActiveCalibrationMatrix(int num)
{
	std::cout << "\n\n*******Setting Active Calibration Matrix Num to: "<< num <<"********"<< std::endl;
	bool ret = true;
	CanMsg CMsg;
	CMsg.setID(m_CanBaseIdentifier | SET_CALIB);
	CMsg.setLength(1);
	CMsg.setAt(num,0);

	ret = m_pCanCtrl->transmitMsg(CMsg, true);

	if (ret) {

		CanMsg replyMsg;
		ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
		if(ret)
		{
			std::cout<<"reply ID: \t"<< std::hex << replyMsg.getID()<<std::endl;
			std::cout<<"reply Length: \t"<<replyMsg.getLength()<<std::endl;
			if(replyMsg.getID() == (m_CanBaseIdentifier | SET_CALIB))
			{
				std::cout<<"Setting Calibration Matrix succeed!"<<std::endl;
				std::cout<<"Calibration Matrix: "<<replyMsg.getAt(0)<<" is Activ!"<<std::endl;
			}
			else
				std::cout<<"Error: Received wrong opcode!"<<std::endl;
		}
		else
			std::cout<<"Error: Receiving Message failed!"<<std::endl;
	}
	else {
		std::cout << "ForceTorqueCtrl::SetActiveCalibrationMatrix(int num): Can not transmit message!" << std::endl;
	}

	return ret;

}

bool ForceTorqueCtrl::SetBaudRate(int value)
{
    std::cout << "\n\n*******Setting Baud Rate value to: "<< value <<"********"<< std::endl;
    bool ret = true;
    CanMsg CMsg;
    CMsg.setID(m_CanBaseIdentifier | SET_BAUD);
    CMsg.setLength(1);
    CMsg.setAt(value,0);

    ret = m_pCanCtrl->transmitMsg(CMsg, true);

    if (ret) {

	CanMsg replyMsg;
	ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
	if(ret)
	{
		std::cout<<"reply ID: \t"<< std::hex << replyMsg.getID()<<std::endl;
		std::cout<<"reply Length: \t"<<replyMsg.getLength()<<std::endl;
		if(replyMsg.getID() == (m_CanBaseIdentifier | SET_BAUD))
		{
			std::cout<<"Setting Baud Rate succeed!"<<std::endl;
			std::cout<<"Send Baud Rate value: "<<CMsg.getAt(0)<<"!"<<std::endl;
		}
		else
			std::cout<<"Error: Received wrong opcode!"<<std::endl;
	}
	else
		std::cout<<"Error: Receiving Message failed!"<<std::endl;
    }
    else {
	std::cout << "ForceTorqueCtrl::SetBaudRate(int value): Can not transmit message!" << std::endl;
    }

    return ret;
}

bool ForceTorqueCtrl::Reset()
{
    std::cout << "\n\n******* Reseting the NETCANOEM ********"<< std::endl;
    bool ret = true;
    CanMsg CMsg;
    CMsg.setID(m_CanBaseIdentifier | RESET);
    CMsg.setLength(0);

    ret = m_pCanCtrl->transmitMsg(CMsg, true);

    if (!ret) {
	std::cout << "ForceTorqueCtrl::Reset(): Can not transmit message!" << std::endl;
	ret = false;
    }
    
    usleep(10000);

    return ret;
}

bool ForceTorqueCtrl::SetBaseIdentifier(int identifier)
{
    std::cout << "\n\n*******Setting Base Identifier value to HEX : " << std::hex << identifier <<" ********"<< std::endl;
    bool ret = true;
    CanMsg CMsg;
    CMsg.setID(m_CanBaseIdentifier | SET_BASEID);
    CMsg.setLength(1);
    CMsg.setAt(identifier,0);

    ret = m_pCanCtrl->transmitMsg(CMsg, true);

    if (ret) {
	CanMsg replyMsg;
	ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
	if(ret)
	{
	    std::cout<<"reply ID: \t"<< std::hex << replyMsg.getID() << std::endl;
	    std::cout<<"reply Length: \t"<<replyMsg.getLength() << std::endl;
	    if(replyMsg.getID() == (m_CanBaseIdentifier | SET_BASEID))
	    {
		std::cout<<"Setting Base Identifier succeed!"<<std::endl;
		std::cout<<"Send Base Identifier value: "<< std::hex << CMsg.getAt(0) <<"!"<<std::endl;
	    }
	    else
		std::cout<<"Error: Received wrong opcode!"<<std::endl;
	}
	else
	    std::cout<<"Error: Receiving Message failed!"<<std::endl;
    }
    else {
	std::cout << "ForceTorqueCtrl::SetBaseIdentifier(int identifier): Can not transmit message!" << std::endl;
    }

    return ret;
}


void ForceTorqueCtrl::ReadCalibrationMatrix()
{
	Eigen::VectorXf vCoef(6);

	//Read Fx coefficients
	ReadMatrix(0, vCoef);
	m_v3FXGain = vCoef;

	//Read Fy coefficients
	ReadMatrix(1, vCoef);
	m_v3FYGain = vCoef;

	//Read Fz coefficients
	ReadMatrix(2, vCoef);
	m_v3FZGain = vCoef;

	//Read Tx coefficients
	ReadMatrix(3, vCoef);
	m_v3TXGain = vCoef;

	//Read Ty coefficients
	ReadMatrix(4, vCoef);
	m_v3TYGain = vCoef;

	//Read Tz coefficients
	ReadMatrix(5, vCoef);
	m_v3TZGain = vCoef;
	SetCalibMatrix();
}

void ForceTorqueCtrl::ReadMatrix(int axis, Eigen::VectorXf& vec)
{
	std::cout << "\n\n*******Read Matrix**********"<<std::endl;
	float statusCode = 0, sg0 = 0.0, sg1 = 0.0, sg2 = 0.0, sg3 = 0.0, sg4 = 0.0, sg5 = 0.0;

	CanMsg CMsg;
	CMsg.setID(m_CanBaseIdentifier | READ_MATRIX);
	CMsg.setLength(1);
	CMsg.setAt(axis,0);

	bool ret = m_pCanCtrl->transmitMsg(CMsg, true);
	if(!ret)
	{
		std::cout<<"Error: Requesting Calibration Matrix!"<<std::endl;
		return;
	}
	
	CanMsg replyMsg;
	bool ret2 = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
	if(ret2)
	{
		std::cout << "reply ID: \t" << std::hex << replyMsg.getID()<<std::endl;
		std::cout << "reply Length: \t" << replyMsg.getLength()<<std::endl;
		std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " "
				      << replyMsg.getAt(2) << " " << replyMsg.getAt(3) << " "
				      << replyMsg.getAt(4) << " " << replyMsg.getAt(5) << " "
				      << replyMsg.getAt(6) << " " << replyMsg.getAt(7) << std::endl;

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
	if(ret2)
	{
		std::cout << "reply ID: \t" << std::hex << replyMsg.getID()<<std::endl;
		std::cout << "reply Length: \t" << replyMsg.getLength()<<std::endl;
		std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " "
				      << replyMsg.getAt(2) << " " << replyMsg.getAt(3) << " "
				      << replyMsg.getAt(4) << " " << replyMsg.getAt(5) << " "
				      << replyMsg.getAt(6) << " " << replyMsg.getAt(7) << std::endl;

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
	if(ret2)
	{
		std::cout << "reply ID: \t" << std::hex << replyMsg.getID()<<std::endl;
		std::cout << "reply Length: \t" << replyMsg.getLength()<<std::endl;
		std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " "
				      << replyMsg.getAt(2) << " " << replyMsg.getAt(3) << " "
				      << replyMsg.getAt(4) << " " << replyMsg.getAt(5) << " "
				      << replyMsg.getAt(6) << " " << replyMsg.getAt(7) << std::endl;

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

	vec[0] = sg0; vec[1] = sg1; vec[2] = sg2; vec[3] = sg3; vec[4] = sg4; vec[5] = sg5;
	std::cout<<"Matix:  SG0: "<<sg0<<" SG1: "<<sg1<<" SG2: "<<sg2<<" SG3: "<<sg3<<" SG4: "<<sg4<<" SG5: "<<sg5<<std::endl;
}

bool ForceTorqueCtrl::ReadFirmwareVersion()
{
	std::cout << "\n\n*******Reading Firmware Version*******"<< std::endl;
	bool ret = true;
	CanMsg CMsg;
	CMsg.setID(m_CanBaseIdentifier | READ_FIRMWARE);
	CMsg.setLength(0);

	ret = m_pCanCtrl->transmitMsg(CMsg, true);

	if (ret)
	{
		CanMsg replyMsg;
		ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
		if(ret)
		{
			std::cout<<"reply ID: \t"<< std::hex << replyMsg.getID()<<std::endl;
			std::cout<<"reply Length: \t"<<replyMsg.getLength()<<std::endl;
			if(replyMsg.getID() == (m_CanBaseIdentifier | READ_FIRMWARE))
			{
				std::cout<<"Reading Firmware Succeed!"<<std::endl;
				std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " "
						  << replyMsg.getAt(2) << " " << replyMsg.getAt(3) << std::endl;
			}
			else {
				std::cout<<"Error: Received wrong opcode!"<<std::endl;
				ret = false;
			}
		}
		else {
			std::cout<<"Error: Receiving Message failed!"<<std::endl;
			ret = false;
		}
	}
	else {
		std::cout<<"Error: Transmiting Message failed!"<<std::endl;
		ret = false;
	}

	return ret;
}

void ForceTorqueCtrl::ReadSGData(int statusCode, double &Fx, double &Fy, double &Fz, double &Tx, double &Ty, double &Tz)
{
    int sg0 = 0, sg1 = 0, sg2 = 0, sg3 = 0, sg4 = 0, sg5 = 0;

    CanMsg CMsg;
    CMsg.setID(m_CanBaseIdentifier | READ_SG);
    CMsg.setLength(0);

    bool ret = m_pCanCtrl->transmitMsg(CMsg, true);
    
    if (!ret) {
	std::cout << "ForceTorqueCtrl::ReadSGData: Error: Transmiting message failed!" << std::endl;
	return;
    }

    CanMsg replyMsg;
    bool ret2 = m_pCanCtrl->receiveMsgTimeout(&replyMsg, -1);
    unsigned char c[2];
    if(ret2)
    {
	if(replyMsg.getID() != (m_CanBaseIdentifier | 0x0)) {
	    
	    std::cout << "ForceTorqueCtrl::ReadSGData: Error: Received wrong opcode (Should be 0x200)!" << std::endl;
	    std::cout << "reply ID: \t" << std::hex << replyMsg.getID()<<std::endl;
	    std::cout << "reply Length: \t" << replyMsg.getLength()<<std::endl;
	    std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " "
				    << replyMsg.getAt(2) << " " << replyMsg.getAt(3) << " "
				    << replyMsg.getAt(4) << " " << replyMsg.getAt(5) << " "
				    << replyMsg.getAt(6) << " " << replyMsg.getAt(7) << std::endl;
	    return;	    
	}

	c[0] = replyMsg.getAt(0); //status code
	c[1] = replyMsg.getAt(1);
	statusCode = (short)((c[0] << 8) | c[1]);
	
	if (statusCode != 0) {	    
	    if (statusCode & 0x4000) {
		std::cout << "ForceTorqueCtrl::ReadSGData: CAN bus error detected!" << std::endl;
		Reset();
		std::cout << "ForceTorqueCtrl::ReadSGData: FTS reseted!" << std::endl;
	    }
	    else {
		std::cout << "ForceTorqueCtrl::ReadSGData: Error: Something is wrong with sensor!" << std::endl;
	    std::cout << std::hex << statusCode << std::endl;
	    }
	}

	c[0] = replyMsg.getAt(2); //sg0
	c[1] = replyMsg.getAt(3);
	sg0 = (short)((c[0] << 8) | c[1]);

	c[0] = replyMsg.getAt(4); //sg2
	c[1] = replyMsg.getAt(5);
	sg2 = (short)((c[0] << 8) | c[1]);

	c[0] = replyMsg.getAt(6); //sg4
	c[1] = replyMsg.getAt(7);
	sg4 = (short)((c[0] << 8) | c[1]);
    }
    else
	return;

    ret2 = m_pCanCtrl->receiveMsgTimeout(&replyMsg, -1);
    if(ret2)
    {
	if(replyMsg.getID() != (m_CanBaseIdentifier | 0x1)) {
	    
	    std::cout<<"ForceTorqueCtrl::ReadSGData: Error: Received wrong opcode (Should be 0x201)!"<<std::endl;
	    std::cout << "reply ID: \t" << std::hex << replyMsg.getID()<<std::endl;
	    std::cout << "reply Length: \t" << replyMsg.getLength()<<std::endl;
	    std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " "
				    << replyMsg.getAt(2) << " " << replyMsg.getAt(3) << " "
				    << replyMsg.getAt(4) << " " << replyMsg.getAt(5) << " "
				    << replyMsg.getAt(6) << " " << replyMsg.getAt(7) << std::endl;
	    return;    
	}

	c[0] = replyMsg.getAt(0); //sg1
	c[1] = replyMsg.getAt(1);
	sg1 = (short)((c[0] << 8) | c[1]);

	c[0] = replyMsg.getAt(2); //sg3
	c[1] = replyMsg.getAt(3);
	sg3 = (short)((c[0] << 8) | c[1]);

	c[0] = replyMsg.getAt(4); //sg5
	c[1] = replyMsg.getAt(5);
	sg5 = (short)((c[0] << 8) | c[1]);
    }
    else
	    return;


//     std::cout<<"\nsg0: "<<sg0<<" sg1: "<<sg1<<" sg2: "<<sg2<<" sg3: "<<sg3<<" sg4: "<<sg4<<" sg5: "<<sg5<<std::endl;

    StrainGaugeToForce(sg0, sg1, sg2, sg3, sg4, sg5);

    Fx = m_vForceData(0); Fy = m_vForceData(1); Fz = m_vForceData(2);
    Tx = m_vForceData(3); Ty= m_vForceData(4); Tz = m_vForceData(5);
}

void ForceTorqueCtrl::StrainGaugeToForce(int& sg0, int& sg1, int& sg2, int& sg3, int& sg4, int& sg5)
{
	Eigen::VectorXf v6SG(6);
	Eigen::VectorXf v6tmp(6);
	Eigen::VectorXf test(6);

	v6SG[0] = sg0; v6SG[1] = sg1; v6SG[2] = sg2; v6SG[3] = sg3; v6SG[4] = sg4; v6SG[5] = sg5;
	//v6SG[0] = 263; v6SG[1] = -272; v6SG[2] = -137; v6SG[3] = 9; v6SG[4] = 275; v6SG[5] = -258;

/*
	v6tmp = v6SG - m_v3StrainGaigeOffset;
	//std::cout<<"\nv6tmp: \n"<< v6tmp <<std::endl;
	//std::cout<<"\nCalibration Matrix: \n"<<m_mXCalibMatrix<<"\n\n";
	//std::cout<<"\nCalibration Matrix Transposed: \n"<<m_mXCalibMatrix.transpose()<<"\n\n";

	test = m_mXCalibMatrix * v6tmp;
	//std::cout << "Test: \n" << test << std::endl;
	m_vForceData = test;
*/
	//std::cout<<"\nCalibration Matrix: \n"<<m_mXCalibMatrix<<"\n\n";
	//std::cout<<"Calibration Matrix: rows: "<<m_mXCalibMatrix.rows()<<" columns: "<<m_mXCalibMatrix.cols()<<std::endl;
	test = m_mXCalibMatrix * v6SG;
	//std::cout<<"test: rows: "<<test.rows()<<" columns: "<<test.cols()<<std::endl;
	//std::cout<<"v6SG: rows: "<<v6SG.rows()<<" columns: "<<v6SG.cols()<<std::endl;
	m_vForceData = test * 0.000001;
	//std::cout << "test: \n" << m_vForceData << std::endl;
	//std::cout<<"m_vForceData: rows: "<<m_vForceData.rows()<<" columns: "<<m_vForceData.cols()<<std::endl;
}

void ForceTorqueCtrl::SetGaugeOffset(float sg0Off, float sg1Off, float sg2Off, float sg3Off, float sg4Off, float sg5Off)
{
	Eigen::VectorXf tmp(6);
	tmp[0] = sg0Off; tmp[1] = sg1Off; tmp[2] = sg2Off; tmp[3] = sg3Off; tmp[4] = sg4Off; tmp[5] = sg5Off;
	m_v3StrainGaigeOffset = tmp;
	//std::cout<<"GaugeOffset: \n"<<m_v3StrainGaigeOffset<<"\n";
	//std::cout<<"GaugeOffset 0: \n"<<tmp[0]<<"\n\n";
}
void ForceTorqueCtrl::SetGaugeGain(float gg0, float gg1, float gg2, float gg3, float gg4, float gg5)
{
	Eigen::VectorXf tmp(6);
	tmp[0] = gg0; tmp[1] = gg1; tmp[2] = gg2; tmp[3] = gg3; tmp[4] = gg4; tmp[5] = gg5;
	m_v3GaugeGain = tmp;
	//std::cout<<"GaugeGain: \n"<<m_v3GaugeGain<<"\n\n";
}

void ForceTorqueCtrl::SetFXGain(float fxg0, float fxg1, float fxg2, float fxg3, float fxg4, float fxg5)
{
	Eigen::VectorXf tmp(6);
	tmp[0] = fxg0; tmp[1] = fxg1; tmp[2] = fxg2; tmp[3] = fxg3; tmp[4] = fxg4; tmp[5] = fxg5;
	m_v3FXGain = tmp;
	//std::cout<<"FXGain: \n"<<m_v3FXGain<<"\n\n";
}
void ForceTorqueCtrl::SetFYGain(float fyg0, float fyg1, float fyg2, float fyg3, float fyg4, float fyg5)
{
	Eigen::VectorXf tmp(6);
	tmp[0] = fyg0; tmp[1] = fyg1; tmp[2] = fyg2; tmp[3] = fyg3; tmp[4] = fyg4; tmp[5] = fyg5;
	m_v3FYGain = tmp;
	//std::cout<<"FYGain: \n"<<m_v3FYGain<<"\n\n";

}
void ForceTorqueCtrl::SetFZGain(float fzg0, float fzg1, float fzg2, float fzg3, float fzg4, float fzg5)
{
	Eigen::VectorXf tmp(6);
	tmp[0] = fzg0; tmp[1] = fzg1; tmp[2] = fzg2; tmp[3] = fzg3; tmp[4] = fzg4; tmp[5] = fzg5;
	m_v3FZGain = tmp;
	//std::cout<<"FZGain: \n"<<m_v3FZGain<<"\n\n";

}
void ForceTorqueCtrl::SetTXGain(float txg0, float txg1, float txg2, float txg3, float txg4, float txg5)
{
	Eigen::VectorXf tmp(6);
	tmp[0] = txg0; tmp[1] = txg1; tmp[2] = txg2; tmp[3] = txg3; tmp[4] = txg4; tmp[5] = txg5;
	m_v3TXGain = tmp;
	//std::cout<<"TXGain: \n"<<m_v3TXGain<<"\n\n";
}
void ForceTorqueCtrl::SetTYGain(float tyg0, float tyg1, float tyg2, float tyg3, float tyg4, float tyg5)
{
	Eigen::VectorXf tmp(6);
	tmp[0] = tyg0; tmp[1] = tyg1; tmp[2] = tyg2; tmp[3] = tyg3; tmp[4] = tyg4; tmp[5] = tyg5;
	m_v3TYGain = tmp;
	//std::cout<<"TYGain: \n"<<m_v3TYGain<<"\n\n";
}
void ForceTorqueCtrl::SetTZGain(float tzg0, float tzg1, float tzg2, float tzg3, float tzg4, float tzg5)
{
	Eigen::VectorXf tmp(6);
	tmp[0] = tzg0; tmp[1] = tzg1; tmp[2] = tzg2; tmp[3] = tzg3; tmp[4] = tzg4; tmp[5] = tzg5;
	m_v3TZGain = tmp;
	//std::cout<<"TZGain: \n"<<m_v3TZGain<<"\n\n";
}

void ForceTorqueCtrl::CalcCalibMatrix()
{
	Eigen::MatrixXf tmp(6, 6);
	tmp(0) = m_v3FXGain[0]/m_v3GaugeGain[0];
	tmp(1) = m_v3FXGain[1]/m_v3GaugeGain[1];
	tmp(2) = m_v3FXGain[2]/m_v3GaugeGain[2];
	tmp(3) = m_v3FXGain[3]/m_v3GaugeGain[3];
	tmp(4) = m_v3FXGain[4]/m_v3GaugeGain[4];
	tmp(5) = m_v3FXGain[5]/m_v3GaugeGain[5];

	tmp(6) = m_v3FYGain[0]/m_v3GaugeGain[0];
	tmp(7) = m_v3FYGain[1]/m_v3GaugeGain[1];
	tmp(8) = m_v3FYGain[2]/m_v3GaugeGain[2];
	tmp(9) = m_v3FYGain[3]/m_v3GaugeGain[3];
	tmp(10) = m_v3FYGain[4]/m_v3GaugeGain[4];
	tmp(11) = m_v3FYGain[5]/m_v3GaugeGain[5];

	tmp(12) = m_v3FZGain[0]/m_v3GaugeGain[0];
	tmp(13) = m_v3FZGain[1]/m_v3GaugeGain[1];
	tmp(14) = m_v3FZGain[2]/m_v3GaugeGain[2];
	tmp(15) = m_v3FZGain[3]/m_v3GaugeGain[3];
	tmp(16) = m_v3FZGain[4]/m_v3GaugeGain[4];
	tmp(17) = m_v3FZGain[5]/m_v3GaugeGain[5];

	tmp(18) = m_v3TXGain[0]/m_v3GaugeGain[0];
	tmp(19) = m_v3TXGain[1]/m_v3GaugeGain[1];
	tmp(20) = m_v3TXGain[2]/m_v3GaugeGain[2];
	tmp(21) = m_v3TXGain[3]/m_v3GaugeGain[3];
	tmp(22) = m_v3TXGain[4]/m_v3GaugeGain[4];
	tmp(23) = m_v3TXGain[5]/m_v3GaugeGain[5];

	tmp(24) = m_v3TYGain[0]/m_v3GaugeGain[0];
	tmp(25) = m_v3TYGain[1]/m_v3GaugeGain[1];
	tmp(26) = m_v3TYGain[2]/m_v3GaugeGain[2];
	tmp(27) = m_v3TYGain[3]/m_v3GaugeGain[3];
	tmp(28) = m_v3TYGain[4]/m_v3GaugeGain[4];
	tmp(29) = m_v3TYGain[5]/m_v3GaugeGain[5];

	tmp(30) = m_v3TZGain[0]/m_v3GaugeGain[0];
	tmp(31) = m_v3TZGain[1]/m_v3GaugeGain[1];
	tmp(32) = m_v3TZGain[2]/m_v3GaugeGain[2];
	tmp(33) = m_v3TZGain[3]/m_v3GaugeGain[3];
	tmp(34) = m_v3TZGain[4]/m_v3GaugeGain[4];
	tmp(35) = m_v3TZGain[5]/m_v3GaugeGain[5];

	m_mXCalibMatrix = tmp;

}
void ForceTorqueCtrl::SetCalibMatrix()
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
