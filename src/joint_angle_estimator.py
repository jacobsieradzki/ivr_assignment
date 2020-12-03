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

	    self.n_tests = 0

	    self.j2t = rospy.Subscriber("/robot/joint2_position_controller/command", Float64, self.j2tc)
	    self.j2v = 0
	    self.j2ns = 0

	    self.j3t = rospy.Subscriber("/robot/joint3_position_controller/command", Float64, self.j3tc)
	    self.j3v = 0
	    self.j3ns = 0

	    self.j4t = rospy.Subscriber("/robot/joint4_position_controller/command", Float64, self.j4tc)
	    self.j4v = 0
	    self.j4ns = 0

	def j2tc(self, msg): self.j2v = msg.data
	def j3tc(self, msg): self.j3v = msg.data
	def j4tc(self, msg): self.j4v = msg.data


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

		joint_2_angle = self.calculate_primary_angle(blue, green, 1, 3)
		joint_3_angle = -self.calculate_primary_angle(blue, green, 0, 4)
		joint_4_angle = self.calculate_secondary_angle(blue, green, red, joint_2_angle)
		
	
	# --- Helper methods to test ---	
	
	# Print the estimated angle, the real angle, the difference and the relevant coordinates.
	def debug_angle(self, calc, real, from_coords, to_coords):
		diff = abs(real-calc)
		print(round(calc, 2), round(real, 2), round(diff, 2), from_coords, to_coords)
		
	# ------------------------------


def main(args):
  icp = joint_angle_estimator()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


