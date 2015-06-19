#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_drones')
import sys
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin

# parent frame name, in which the virtual drone moves
global_frame = "world"
drone_frame = "virtual_drone"

# velocity scaling
ref_vel_scale = 2.0
ref_yaw_scale = 2.0

def saturate(inp,limit):
  # limit quantity to [-limit,limit]
  out = inp
  if inp>limit:
    out=limit
  elif inp<-limit:
    out = -limit
  return out

def refUpdate():
  # need to explicitly ref global as I'm writing to it
  global ref_transform
  # extract Euler angles from current pose
  rpyAngles = euler_from_quaternion(ref_transform.rotation)
  # and I just need the yaw
  yawAngle = rpyAngles[2]
  # propagate the reference
  ref_transform.translation.x = ref_transform.translation.x + delta_t*ref_vel_scale*(ref_velocity.linear.x*cos(yawAngle)-ref_velocity.linear.y*sin(yawAngle))
  ref_transform.translation.y = ref_transform.translation.y + delta_t*ref_vel_scale*(ref_velocity.linear.x*sin(yawAngle)+ref_velocity.linear.y*cos(yawAngle))
  ref_transform.translation.z = ref_transform.translation.z + delta_t*ref_vel_scale*ref_velocity.linear.z
  # update the rotation as well
  yawAngle = yawAngle + delta_t*ref_yaw_scale*ref_velocity.angular.z
  ref_transform.rotation = quaternion_from_euler(0.0, 0.0, yawAngle)
  # publish the reference as a transform
  pub_ref_tf.sendTransform((ref_transform.translation.x, ref_transform.translation.y, ref_transform.translation.z),
                           ref_transform.rotation,
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
# default orientation
ref_transform.rotation = quaternion_from_euler(0.0, 0.0, 0.0)
# and the reference velocity
ref_velocity = Twist()

# update rate and time step
updateRateHz = 10
delta_t = 1./updateRateHz
rate = rospy.Rate(updateRateHz)

while not rospy.is_shutdown():
  refUpdate()
  rate.sleep()
