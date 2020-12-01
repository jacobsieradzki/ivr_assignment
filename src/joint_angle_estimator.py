#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import math
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class joint_angle_estimator:

	def __init__(self):
	    rospy.init_node('joint_angle_estimator', anonymous=True)
	    self.sync = message_filters.ApproximateTimeSynchronizer([
	      message_filters.Subscriber("/jake/vision_coordinates/blue", Float64MultiArray),
	      message_filters.Subscriber("/jake/vision_coordinates/green", Float64MultiArray),
	      message_filters.Subscriber("/jake/vision_coordinates/red", Float64MultiArray),
	    ], 10, 0.1, allow_headerless=True)
	    self.sync.registerCallback(self.coordinate_callbak)



	def coordinate_callbak(self, blue_coords, green_coords, red_coords):
		print(blue_coords.data, green_coords.data, red_coords.data)


def main(args):
  icp = joint_angle_estimator()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


