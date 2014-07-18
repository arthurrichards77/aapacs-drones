#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_drones')
import sys
import rospy
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from sensor_msgs.msg import Imu

acc_threshold = 0.8
pos_bump = 0.3

def viconCallback(data):
  # need to explicitly ref message store global as I'm writing to it
  global last_tf
  # just store in case I need it
  last_tf = data.transform

def imuCallback(data):
  # default ref transform
  ref_tf = last_tf
  # suppress reflex under 0.3m altitude
  if last_tf.translation.z > 0.3:
    # check for X collision
    if data.linear_acceleration.x > acc_threshold:
      # reflex back
      ref_tf.translation.x = ref_tf.translation.x + pos_bump
      pub_ref_tf.publish(ref_tf)
      print 'BUMP! reflex forward'
    elif data.linear_acceleration.x < -acc_threshold:
      # reflex fowards
      ref_tf.translation.x = ref_tf.translation.x - pos_bump
      pub_ref_tf.publish(ref_tf)
      print 'BUMP! reflex backward'
    # check for y collision
    elif data.linear_acceleration.y > acc_threshold:
      # reflex back
      ref_tf.translation.y = ref_tf.translation.y + pos_bump
      pub_ref_tf.publish(ref_tf)
      print 'BUMP! reflex right'
    elif data.linear_acceleration.y < -acc_threshold:
      # reflex fowards
      ref_tf.translation.y = ref_tf.translation.y - pos_bump
      pub_ref_tf.publish(ref_tf)
      print 'BUMP! reflex left'

rospy.init_node('collision_reflex', anonymous=True)
# subscribes to vicon and ARDrone IMU
sub_vicondata = rospy.Subscriber('drone', TransformStamped, viconCallback)
sub_ref_imu = rospy.Subscriber('ardrone/imu', Imu, imuCallback)
# publishes to "ref_tf" channel to command drone position
pub_ref_tf = rospy.Publisher('ref_tf', Transform)

# and for reference position
last_tf = Transform()

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
