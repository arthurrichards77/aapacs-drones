#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_drones')
import sys
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped

# parent frame name, in which the virtual drone moves
global_frame = "world"
drone_frame = "virtual_drone"

def saturate(inp,limit):
  # limit quantity to [-limit,limit]
  out = inp
  if inp>limit:
    out=limit
  elif inp<-limit:
    out = -limit
  return out

def refUpdate():
  # need to explicitly ref message store global as I'm writing to it
  global ref_transform
  # propagate the reference
  ref_vel_scale = 2.0
  ref_transform.translation.x = ref_transform.translation.x + delta_t*ref_velocity.linear.x*ref_vel_scale
  ref_transform.translation.y = ref_transform.translation.y + delta_t*ref_velocity.linear.y*ref_vel_scale
  ref_transform.translation.z = ref_transform.translation.z + delta_t*ref_velocity.linear.z*ref_vel_scale
  # publish the reference as a transform
  pub_ref_tf.sendTransform((ref_transform.translation.x, ref_transform.translation.y, ref_transform.translation.z),
                           (0.0, 0.0, 0.0, 1.0),
                           rospy.Time.now(),
                           drone_frame, global_frame)

def refVelCallback(data):
  global ref_velocity
  ref_velocity = data

def reftfCallback(data):
  global ref_velocity
  # set reference movement to zero
  ref_velocity = Twist()
  # and set position to whatever was received
  ref_transform.translation.x = data.transform.translation.x
  ref_transform.translation.y = data.transform.translation.y
  ref_transform.translation.z = data.transform.translation.z

rospy.init_node('virtual_drone', anonymous=True)
sub_ref_vel = rospy.Subscriber('ref_vel', Twist, refVelCallback)
sub_ref_pos = rospy.Subscriber('ref_tf', TransformStamped, reftfCallback)
pub_ref_tf = tf.TransformBroadcaster()

# and for reference position
ref_transform = Transform()
# default altitude
ref_transform.translation.z = 1.2
# and the reference velocity
ref_velocity = Twist()

# update rate and time step
updateRateHz = 10
delta_t = 1./updateRateHz
rate = rospy.Rate(updateRateHz)

while not rospy.is_shutdown():
  refUpdate()
  rate.sleep()
