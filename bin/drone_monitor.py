#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_drones')
import sys
import rospy
import cv2
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ardrone_autonomy.msg import Navdata

class image_converter:

  def __init__(self):

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image",Image,self.callback)
    self.nav_sub = rospy.Subscriber("ardrone/navdata",Navdata,self.navcallback)
    self.last_batt = 100

  def navcallback(self,data):
    # store the battery percentage
    self.last_batt = data.batteryPercent

  def callback(self,data):
    try:
      cv1_image = self.bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
      print e

    # conversion from fuerte bridge format to new opencv
    cv_image = numpy.asarray(cv1_image)

    # circle tag location, if any seen    
    screen_msg="Battery = %i percent" % self.last_batt
    cv2.putText(cv_image, screen_msg, (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)

    # show the image
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
