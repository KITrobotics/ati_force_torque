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


class ForceTorqueNode
{
public:

    ForceTorqueNode();
    
    bool initFts();
    bool srvCallback_SetBaudRate(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res);
    bool srvCallback_SetBaseIdentifier(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res);
    // create a handle for this node, initialize node
    ros::NodeHandle nh_;

private:
    // CAN parameters
    int deviceType;
    std::string devicePath;
    int deviceBaudrate;
    int deviceBaseIdentifier;
    int futureBaudrate;
    int futureIdentifier;

    // service servers
    ros::ServiceServer srvServer_SetBaudRate_;
    ros::ServiceServer srvServer_SetBaseIdentifier_;
    
    bool m_isInitialized;
    ForceTorqueCtrl* p_Ftc;
};

ForceTorqueNode::ForceTorqueNode()
{

	m_isInitialized = false;

	srvServer_SetBaudRate_ = nh_.advertiseService("SetBaudRate", &ForceTorqueNode::srvCallback_SetBaudRate, this);
	srvServer_SetBaseIdentifier_ = nh_.advertiseService("SetBaseIdentifier", &ForceTorqueNode::srvCallback_SetBaseIdentifier, this);

	// Read data from parameter server
	nh_.param<int>("device/type", deviceType, -1);
	nh_.param<std::string>("device/path", devicePath, "");
	nh_.param<int>("device/baudrate", deviceBaudrate, -1);
	nh_.param<int>("device/base_identifier", deviceBaseIdentifier, -1);
	nh_.param<int>("device/future_baudrate", futureBaudrate, ATI_CAN_BAUD_250K);
	nh_.param<int>("device/future_identifier", futureIdentifier, 0x20);

	p_Ftc = new ForceTorqueCtrl(deviceType, devicePath, deviceBaudrate, deviceBaseIdentifier);
	
// 	usleep(10000);
	initFts();

}

bool ForceTorqueNode::initFts()
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

bool ForceTorqueNode::srvCallback_SetBaudRate(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res) {
 
    if (m_isInitialized) {
	
	if (p_Ftc->SetBaudRate(futureBaudrate)) {
	    
	    ROS_INFO("New baud rate successfully set to: %d", futureBaudrate);
	
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

bool ForceTorqueNode::srvCallback_SetBaseIdentifier(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res) {
 
    if (m_isInitialized) {
	
	if (p_Ftc->SetBaseIdentifier(futureIdentifier)) {
	    
	    ROS_INFO("New base identifier successfully set to HEX: %x", futureIdentifier);
	
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

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "forcetorque_set_config");
  ForceTorqueNode ftn;

  ROS_INFO("ForceTorque Sensor Config Node running.");
  
  ros::spin();

  return 0;
}