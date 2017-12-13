#include <ati_force_torque/force_torque_sensor_handle.h>
#include <hardware_interface/sensor_hw.h>

namespace ati_force_torque
{

class ForceTorqueSensorHW : public hardware_interface::SensorHW
{
public:
  ForceTorqueSensorHW();
  virtual ~ForceTorqueSensorHW(){};
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  virtual void read(const ros::Time& time, const ros::Duration& period);
  
  virtual bool stop();
  virtual bool recover();

protected:  
  hardware_interface::ForceTorqueSensorInterface fts_interface_;
  std::string fts_name, fts_transform_frame;
  ForceTorqueSensorHandle* ftsh_;
};

ForceTorqueSensorHW::ForceTorqueSensorHW()
{
}

bool ForceTorqueSensorHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
  ROS_INFO("in init ati_ft");
  
  // Populate hardware interfaces
  robot_hw_nh.param<std::string>("FTS/fts_name", fts_name, "ATI_45_Mini");
  robot_hw_nh.param<std::string>("Node/transform_frame", fts_transform_frame, "fts_transform_frame");
  
  ftsh_ = new ForceTorqueSensorHandle(robot_hw_nh, fts_name, fts_transform_frame);
  fts_interface_.registerHandle(*ftsh_);
  
  registerInterface(&fts_interface_);

  return true;
}


void ForceTorqueSensorHW::read(const ros::Time& time, const ros::Duration& period)
{
  double* force; 
  force = ftsh_->getForce();  
  double* torque;
  torque = ftsh_->getTorque();  
}

 bool ForceTorqueSensorHW::stop() 
 {
     ROS_INFO("to be implemented"); 
     return true;
 }
 
 bool ForceTorqueSensorHW::recover() 
 {
     ROS_INFO("to be implemented"); 
     return true;
 };
}


