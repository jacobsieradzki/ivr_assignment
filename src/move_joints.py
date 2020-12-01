#!/usr/bin/env python3
import sys
import roslib
import rospy
import numpy as np
from std_msgs.msg import Float64

class move_joints:

	def __init__(self):
		rospy.init_node('move_joints', anonymous=True)
		self.rate = rospy.Rate(30) # 30Hz
		self.joints = [
			{
				"enabled": True,
				"constant": 15,
				"publisher": rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
			},
			{
				"enabled": True,
				"constant": 18,
				"publisher": rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
			},
			{
				"enabled": True,
				"constant": 20,
				"publisher": rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
			}
		]


	# Calculate angle for joint in form:
	#   pi/2 * sin(pi/a * t)
	# Where:
	#   t denotes time
	#   a is a constant, specific for joint n
	def calculate_joint_angle(self, a, t):
		return (np.pi / 2) * np.sin((np.pi / a) * t)


	# Begin movement
	def begin_move(self):
		t0 = rospy.get_time()

		while not rospy.is_shutdown():
			cur_time = np.array([rospy.get_time()]) - t0

			for joint in self.joints:				
				angle = Float64()

				if joint["enabled"] is True:
					angle.data = self.calculate_joint_angle(joint["constant"], cur_time)
				else:
					angle.data = 0

				joint["publisher"].publish(angle)

			self.rate.sleep()



def main(args):
	mj = move_joints()
	try:
		mj.begin_move()
	except KeyboardInterrupt:
		print("Shutting down...")

if __name__ == '__main__':
	main(sys.argv)