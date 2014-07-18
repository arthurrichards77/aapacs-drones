#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_drones')
import sys
import rospy
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

def inpVelCallback(data):
  # record that there was one
  global currState
  # disarm trigger if armed
  if currState==1:
    currState=0
    # and pass along the message
    pub_Cmd.publish(data)
  elif currState==0:
    # just pass along the message
    pub_Cmd.publish(data)

def navCallback(data):
  # only used in state 3 (landing)
  if currState==3:
    # if I'm close to the ground, land anyway
    if data.altd<200:
      pub_land.publish(Empty())
    else:
      # still tracking OK - run feedback
      cmd_twist = Twist()
      # linear feedback on yaw
      cmd_twist.linear.z = -0.0001*data.altd
      # limit in [-0.5,0.5]
      if cmd_twist.linear.z>0.5:
        cmd_twist.linear.z = 0.5
      elif cmd_twist.linear.z<-0.5:
        cmd_twist.linear.z = -0.5
      # send message
      pub_Cmd.publish(cmd_twist)
  
# setup node and subs/pubs
rospy.init_node('drone_follow', anonymous=False)
sub_Navdata = rospy.Subscriber('ardrone/navdata', Navdata, navCallback)
sub_cmd_vel = rospy.Subscriber('control_vel', Twist, inpVelCallback)
pub_Cmd = rospy.Publisher('cmd_vel', Twist)
pub_land = rospy.Publisher('ardrone/land', Empty)

# state machine variable
currState = 0

# run at 1Hz
r = rospy.Rate(0.5)

while not rospy.is_shutdown():
  r.sleep()
  if currState==0:
    # arm trigger for input timeout
    currState=1
  elif currState==1:
    # no input since arm - takeover
    print 'Timeout!\r\n'
    currState=2
    # start reverse
    rev_cmd = Twist()
    rev_cmd.linear.x = -0.2
    pub_Cmd.publish(rev_cmd)
  elif currState==2:
    # end of reverse: start landing
    print 'Landing\r\n'
    currState=3

