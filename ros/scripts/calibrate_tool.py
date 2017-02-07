#!/usr/bin/env python

from subprocess import call
from math import sqrt

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ati_force_torque.srv import CalculateAverageMasurement
from geometry_msgs.msg import Vector3


def calibrate_tool():
    
    rospy.init_node('calibrate_tool')    
    trajectory_pub = rospy.Publisher('/arm/joint_trajectory_controller/command', JointTrajectory, latch=True, queue_size=1)
    average_measurements_srv = rospy.ServiceProxy('/arm/CalculateAverageMasurement', CalculateAverageMasurement)
    
    joint_names = rospy.get_param('/arm/joint_names')

    tool_name = rospy.get_param('~tool_name')

    # Posees
    poses = [[0.0, 0.0, 1.5707963, 0.0, -1.5707963, 0.0],
                   [0.0, 0.0, 1.5707963, 0.0, 1.5707963, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 1.5707963, 0.0]]

    measurement = []
    
    for i in range(0,len(poses)):  
        trajectory = JointTrajectory()
        point = JointTrajectoryPoint()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.joint_names = joint_names
        
        point.time_from_start = rospy.Duration(5.0)
        point.positions = poses[i]
        
        trajectory.points.append(point)
        trajectory_pub.publish(trajectory)
        rospy.loginfo("Going to position: " + str(poses[i]))
        
        rospy.sleep(10.0)
        
        rospy.loginfo("Calculating offsets.")
        ret = average_measurements_srv(100, 0.01)
        #ret = average_measurements_srv(50, 0.1, "world_frame")

        measurement.append(ret.measurement)

    CoG = Vector3()

    Fg = (measurement[0].force.z - measurement[1].force.z)/2.0;
    CoG.z = (sqrt(measurement[2].torque.x*measurement[2].torque.x + measurement[2].torque.y*measurement[2].torque.y)) / Fg;

    rospy.loginfo("Setting parametes for tool: " + tool_name)

    rospy.set_param('/temp/tool/CoG/x', CoG.x)
    rospy.set_param('/temp/tool/CoG/y', CoG.y)
    rospy.set_param('/temp/tool/CoG/z', CoG.z)
    rospy.set_param('/temp/tool/force', Fg)
    
    

    call("rosparam dump -v `rospack find iirob_description`/tools/urdf/" + tool_name + "/gravity.yaml /temp/tool", shell=True)


if __name__ == "__main__":
    
    try:
        calibrate_tool()
    except rospy.ROSInterruptException:
        pass
