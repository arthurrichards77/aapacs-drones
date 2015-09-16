#!/usr/bin/env python
import rospy
import numpy
import math
from geometry_msgs.msg import Twist

translation_input = Twist()


def translation_callback(data):
  global translation_input
  translation_input = data


def callback(data):
  # Publish rotational rate in translation_input
  output = translation_input
  output.angular.z = data.z
  pub.publish(output)


# Node setup
rospy.init_node('overwrite_yaw', anonymous=False)
pub = rospy.Publisher("cmd_vel_out", Twist, queue_size=1)
rospy.Subscriber('cmd_vel_in',Twist,translation_callback)
rospy.Subscriber('cmd_yaw_in',Twist,callback)
rospy.spin()

