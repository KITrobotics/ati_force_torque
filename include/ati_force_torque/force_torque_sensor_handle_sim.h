#ifndef FORCETORQUESENSORHANDLESIM_INCLUDEDEF_H
#define FORCETORQUESENSORHANDLESIM_INCLUDEDEF_H


#include <hardware_interface/force_torque_sensor_interface.h>
#include <ati_force_torque/force_torque_sensor_sim.h>

class ForceTorqueSensorHandleSim : public ForceTorqueSensorSim, public hardware_interface::ForceTorqueSensorHandle
{
public:
    ForceTorqueSensorHandleSim(ros::NodeHandle &nh, std::string sensor_name, std::string output_frame);

private:
    void updateFTData(const ros::TimerEvent &event);

    // Arrays for hardware_interface
    double interface_force_[3];
    double interface_torque_[3];
};


#endif