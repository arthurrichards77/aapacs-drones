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
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

# mode - "velocity" or "trajectory"
movement_mode = 'velocity'

# parent frame name, in which the virtual drone moves
global_frame = "world"
drone_frame = "virtual_drone"

# velocity scaling
ref_vel_scale = 2.0
ref_yaw_scale = 2.0

# joint state message
js_msg = JointState()
js_msg.name=["flightx","flighty","flightz","yaw"]
js_msg.position = [0.0, 0.0, 0.0, 0.0]
js_msg.velocity = [0.0, 0.0, 0.0, 0.0]
js_msg.effort = [0.0, 0.0, 0.0, 0.0]

# executing trajectory
ref_traj = JointTrajectory()

def saturate(inp,limit):
  # limit quantity to [-limit,limit]
  out = inp
  if inp>limit:
    out=limit
  elif inp<-limit:
    out = -limit
  return out

def trajUpdate():
  return(0,0,0,0)

def velUpdate():
  # update for reference in velocity mode
  # need to explicitly ref global as I'm writing to it
  global ref_transform
  global js_msg
  # extract Euler angles from current pose
  rpyAngles = euler_from_quaternion(ref_transform.rotation)
  # and I just need the yaw
  yawAngle = rpyAngles[2]
  # propagate the reference
  x = ref_transform.translation.x + delta_t*ref_vel_scale*(ref_velocity.linear.x*cos(yawAngle)-ref_velocity.linear.y*sin(yawAngle))
  y = ref_transform.translation.y + delta_t*ref_vel_scale*(ref_velocity.linear.x*sin(yawAngle)+ref_velocity.linear.y*cos(yawAngle))
  z = ref_transform.translation.z + delta_t*ref_vel_scale*ref_velocity.linear.z
  # update the rotation as well
  yawAngle = yawAngle + delta_t*ref_yaw_scale*ref_velocity.angular.z
  # return the new joint states
  return (x,y,z,yawAngle)

def refUpdate():
  # need to explicitly ref global as I'm writing to it
  global ref_transform
  global js_msg
  # update according to what mode we're in
  if movement_mode == 'velocity':
    (x,y,z,yawAngle)=velUpdate()
  elif movement_mode == 'trajectory':
    (x,y,z,yawAngle)=trajUpdate()

  # put the new states back in the transform memory
  ref_transform.translation.x = x;
  ref_transform.translation.y = y;
  ref_transform.translation.z = z;
  ref_transform.rotation = quaternion_from_euler(0.0, 0.0, yawAngle)

  # publish the reference as a transform
  # pub_ref_tf.sendTransform((ref_transform.translation.x, ref_transform.translation.y, ref_transform.translation.z),
  #                         ref_transform.rotation,
  #                         rospy.Time.now(),
  #                         drone_frame, global_frame)

  # and as a set of joint states for the URDF drone
  js_msg.header.stamp = rospy.Time.now()
  js_msg.position = [ref_transform.translation.x, ref_transform.translation.y, ref_transform.translation.z, yawAngle]
  pub_joint_states.publish(js_msg)

def refVelCallback(data):
  global ref_velocity
  global movement_mode
  movement_mode = 'velocity'
  ref_velocity = data

def trajCallback(data):
  global ref_traj
  global movement_mode
  movement_mode = 'trajectory'
  ref_traj = data

def reftfCallback(data):
  global ref_velocity
  global movement_mode
  movement_mode = 'velocity'
  # set reference movement to zero
  ref_velocity = Twist()
  # and set position to whatever was received
  ref_transform.translation.x = data.transform.translation.x
  ref_transform.translation.y = data.transform.translation.y
  ref_transform.translation.z = data.transform.translation.z

rospy.init_node('virtual_drone', anonymous=True)
sub_ref_vel = rospy.Subscriber('cmd_vel', Twist, refVelCallback)
sub_ref_vel = rospy.Subscriber('cmd_traj', JointTrajectory, trajCallback)
sub_ref_pos = rospy.Subscriber('cmd_tf', TransformStamped, reftfCallback)
pub_ref_tf = tf.TransformBroadcaster()
pub_joint_states = rospy.Publisher('/joint_states', JointState)

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
