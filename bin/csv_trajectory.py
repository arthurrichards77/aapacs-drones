#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_drones')
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import csv

def load_csv_traj(file_name):
  # prepare empty trajectory
  traj = JointTrajectory()
  # open the file
  with open(file_name, 'rb') as csv_file:
    traj_reader = csv.reader(csv_file)
    # get first row for names
    first_row = traj_reader.next()
    assert(first_row[0]=='time_from_start')
    traj.joint_names=first_row[1:]
    # then subsequent rows
    for row in traj_reader:
      assert(len(row)==len(first_row))
      new_point = JointTrajectoryPoint()
      new_point.time_from_start=rospy.Duration(float(row[0]))
      new_point.positions = [float(p) for p in row[1:]]
      traj.points += [new_point]
  return(traj)

# default file name
file_name='traj.csv'

ref_traj=load_csv_traj(file_name)

pub_traj = rospy.Publisher('cmd_traj', JointTrajectory)
rospy.init_node('token_trajectory', anonymous=True)

# update rate and time step
updateRateHz = 0.01
rate = rospy.Rate(updateRateHz)
  
while not rospy.is_shutdown():
  rospy.loginfo("Sending trajectory")
  pub_traj.publish(ref_traj)
  rate.sleep()
