#include <ati_force_torque/force_torque_sensor_handle_sim.h>

ForceTorqueSensorHandleSim::ForceTorqueSensorHandleSim(ros::NodeHandle& nh, std::string sensor_name, std::string output_frame) :
    ForceTorqueSensorSim(nh), hardware_interface::ForceTorqueSensorHandle(sensor_name, output_frame, interface_force_, interface_torque_)
{
    transform_frame_ = output_frame;
}

void ForceTorqueSensorHandleSim::updateFTData(const ros::TimerEvent& event)
{
    interface_force_[0] = threshold_filtered_force.wrench.force.x*40;
    interface_force_[1] = threshold_filtered_force.wrench.force.y*55;
    interface_force_[2] = threshold_filtered_force.wrench.force.z;

    interface_torque_[0] = threshold_filtered_force.wrench.torque.x;
    interface_torque_[1] = threshold_filtered_force.wrench.torque.y;
    interface_torque_[2] = threshold_filtered_force.wrench.torque.z*10;
}
