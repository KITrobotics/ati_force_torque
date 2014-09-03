#include <stdint.h>
typedef unsigned char uint8_t;
#include <inttypes.h>
#include <iostream>
#include <ros/ros.h>
#include <cob_forcetorque/ForceTorqueCtrl.h>

#include <cob_srvs/Trigger.h>

#include <math.h>
#include <iostream>
#define PI 3.14159265


class ForceTorqueConfig
{
public:

    ForceTorqueConfig();
    
    bool initFts();
    bool srvCallback_SetBaudRate(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res);
    bool srvCallback_SetBaseIdentifier(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res);
    bool srvCallback_Reset(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res);
    // create a handle for this node, initialize node
    ros::NodeHandle nh_;

private:
    // CAN parameters
    int canType;
    std::string canPath;
    int canBaudrate;
    int ftsBaseID;
    int ftsFutureBaudrate;
    int ftsFutureBaseID;

    // service servers
    ros::ServiceServer srvServer_SetBaudRate_;
    ros::ServiceServer srvServer_SetBaseIdentifier_;
    ros::ServiceServer srvSever_Reset_;
    
    bool m_isInitialized;
    ForceTorqueCtrl* p_Ftc;
};

ForceTorqueConfig::ForceTorqueConfig()
{

	m_isInitialized = false;

	srvServer_SetBaudRate_ = nh_.advertiseService("SetBaudRate", &ForceTorqueConfig::srvCallback_SetBaudRate, this);
	srvServer_SetBaseIdentifier_ = nh_.advertiseService("SetBaseIdentifier", &ForceTorqueConfig::srvCallback_SetBaseIdentifier, this);
	srvSever_Reset_ = nh_.advertiseService("Reset", &ForceTorqueConfig::srvCallback_Reset, this);

	// Read data from parameter server
	nh_.param<int>("CAN/type", canType, -1);
	nh_.param<std::string>("CAN/path", canPath, "");
	nh_.param<int>("CAN/baudrate", canBaudrate, -1);
	nh_.param<int>("FTS/base_identifier", ftsBaseID, -1);
	nh_.param<int>("FTS/future_baudrate", ftsFutureBaudrate, ATI_CAN_BAUD_250K);
	nh_.param<int>("FTS/future_base_id", ftsFutureBaseID, 0x20);

	p_Ftc = new ForceTorqueCtrl(canType, canPath, canBaudrate, ftsBaseID);
	
	initFts();

}

bool ForceTorqueConfig::initFts()
{
    if(!m_isInitialized)
    {
	// read return init status and check it!
	if (p_Ftc->Init()) {
	    ROS_INFO("FTC initialized");
	    m_isInitialized = true;
	}
	else {
	    m_isInitialized = false;
	    ROS_INFO("FTC initialisation failed");
	}
    }
    return m_isInitialized;
}

bool ForceTorqueConfig::srvCallback_SetBaudRate(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res) {
 
    if (m_isInitialized) {
	
	if (p_Ftc->SetBaudRate(ftsFutureBaudrate)) {
	    
	    ROS_INFO("New baud rate successfully set to: %d", ftsFutureBaudrate);
	
	res.success.data = true;
	res.error_message.data = "All good, you are nice person! :)";
	}
	else {
	    res.success.data = false;
	    res.error_message.data = "Could not set baud rate :)";
	}
    }
    else {
	res.success.data = false;
	res.error_message.data = "FTS not initialised! :/";
    }
    
    return true;    
}

bool ForceTorqueConfig::srvCallback_SetBaseIdentifier(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res) {
 
    if (m_isInitialized) {
	
	if (p_Ftc->SetBaseIdentifier(ftsFutureBaseID)) {
	    
	    ROS_INFO("New base identifier successfully set to HEX: %x", ftsFutureBaseID);
	
	res.success.data = true;
	res.error_message.data = "All good, you are nice person! :)";
	}
	else {
	    res.success.data = false;
	    res.error_message.data = "Could not set base identifier :)";
	}
    }
    else {
	res.success.data = false;
	res.error_message.data = "FTS not initialised! :/";
    }
    
    return true;    
}

bool ForceTorqueConfig::srvCallback_Reset(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res) {
    
    ROS_WARN("Going to reset NETCANOEM!");
    
    if (p_Ftc->Reset()) {
	res.success.data = true;
	res.error_message.data = "Reset succeded!";
    }
    else {
	res.success.data = false;
	res.error_message.data = "Reset NOT succeded!";
    }
    
    return true;   
}
    

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "forcetorque_config");
  ForceTorqueConfig ftn;

  ROS_INFO("ForceTorque Config Node running.");
  
  ros::spin();

  return 0;
}