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
	    self.sync.registerCallback(self.coordinate_callback)

	    self.joint_2_pub = rospy.Publisher("/jake/vision_coordinates/joint2", Float64, queue_size=10)
	    self.joint_3_pub = rospy.Publisher("/jake/vision_coordinates/joint3", Float64, queue_size=10)
	    self.joint_4_pub = rospy.Publisher("/jake/vision_coordinates/joint4", Float64, queue_size=10)


	# Used for the blue to green joint angle for rotation about the x or y axis.
	def calculate_primary_angle(self, from_coord, to_coord, opp_axis, adj_axis):
		o = (to_coord[opp_axis] - from_coord[opp_axis])
		a = (to_coord[adj_axis] - from_coord[adj_axis])

		joint_angle = np.pi/2
		if not a == 0:
			joint_angle = math.atan2(o, a)
		
		return max(-np.pi/2, min(joint_angle, np.pi/2))
	
	
	# Helper function for the green to red joint angle to transform the [x,y,z,yz,xz] coordinates into [x,y,z]
	def transform_secondary_coordinates(self, coordinates):
		return np.array([coordinates[0], coordinates[1], coordinates[3]])
	
	
	# Used for the green to red joint angle for rotation about the x axis.	
	def calculate_secondary_angle(self, blue, green, red, joint_2_angle):
		b2g = self.transform_secondary_coordinates(blue) - self.transform_secondary_coordinates(green)
		b2g_mag = np.linalg.norm(b2g)
		g2r = self.transform_secondary_coordinates(green) - self.transform_secondary_coordinates(red)
		g2r_mag = np.linalg.norm(g2r)
		
		return np.arccos(np.dot(b2g, g2r) / (b2g_mag*g2r_mag))


	# All coordinates in form: [x, y, z, yz, xz]
	def coordinate_callback(self, blue_message, green_message, red_message):

		blue = np.asarray(blue_message.data)
		green = np.asarray(green_message.data)
		red = np.asarray(red_message.data)

		joint_2_angle = Float64()
		joint_2_angle.data = self.calculate_primary_angle(blue, green, 1, 3)
		self.joint_2_pub.publish(joint_2_angle)
		joint_3_angle = Float64()
		joint_3_angle.data = -self.calculate_primary_angle(blue, green, 0, 4)
		self.joint_3_pub.publish(joint_3_angle)
		joint_4_angle = Float64()
		joint_4_angle.data = self.calculate_secondary_angle(blue, green, red, joint_2_angle)
		self.joint_4_pub.publish(joint_4_angle)


def main(args):
  icp = joint_angle_estimator()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


