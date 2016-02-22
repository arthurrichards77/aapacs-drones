#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_drones')
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

# make a trivial trajectory
ref_traj = JointTrajectory()
ref_traj.joint_names=["flightx","flighty","flightz","yaw"]
# first point
point1 = JointTrajectoryPoint()
point1.positions = [0.0, 0.0, 0.0, 0.0]
point1.time_from_start = rospy.Duration(1.0)
# second point
point2 = JointTrajectoryPoint()
point2.positions = [1.0, 0.0, 1.0, 3.0]
point2.time_from_start = rospy.Duration(11.0)
# second point
point3 = JointTrajectoryPoint()
point3.positions = [1.0, 1.0, 1.0, -3.0]
point3.time_from_start = rospy.Duration(21.0)
# combine the points
ref_traj.points = [point1, point2, point3]

pub_traj = rospy.Publisher('cmd_traj', JointTrajectory)
rospy.init_node('token_trajectory', anonymous=True)

# update rate and time step
updateRateHz = 0.01
rate = rospy.Rate(updateRateHz)
  
while not rospy.is_shutdown():
  rospy.loginfo("Sending trajectory")
  pub_traj.publish(ref_traj)
  rate.sleep()
