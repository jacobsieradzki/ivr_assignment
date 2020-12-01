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


class image_coordinate_processor:

  def __init__(self):
    rospy.init_node('image_coordinate_processor', anonymous=True)
    self.sync = message_filters.ApproximateTimeSynchronizer([
      message_filters.Subscriber("/camera1/robot/image_raw", Image),
      message_filters.Subscriber("/camera2/robot/image_raw", Image)
    ], 10, 0.1, allow_headerless=True)
    self.sync.registerCallback(self.camera_sub)

    self.blue_coord_pub = rospy.Publisher("/jake/vision_coordinates/blue", Float64MultiArray, queue_size=10)
    self.green_coord_pub = rospy.Publisher("/jake/vision_coordinates/green", Float64MultiArray, queue_size=10)
    self.red_coord_pub = rospy.Publisher("/jake/vision_coordinates/red", Float64MultiArray, queue_size=10)

    self.bridge = CvBridge()



  # Returns coordinates of joint [x,y] format, where
  #   hsv               image from camera in HSV color format
  #   lower_bound_val   lower bound of the Value in HSV to detect the color of the joint
  #   upper_bound_val   upper bound of the Value in HSV to detect the color of the joint
  # 
  # Returns None if the joint cannot be seen in this camera angle.
  def find_ball_2d_coordinates(self, hsv, lower_bound_val, upper_bound_val):
    lower = np.array([lower_bound_val, 50, 50])
    upper = np.array([upper_bound_val, 255, 255])

    mask = cv2.inRange(hsv, lower, upper)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
      M = cv2.moments(contours[0])
      if M['m00'] == 0:
        return None # Joint cannot be seen
      else:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return [cx, cy]
    else:
      return None



  # Returns [x,y,z] coordinates of joint relative to base, where:
  #   yz_hsv           image from camera1 in HSV color format, where the camera angle is orthogonal to the yz plane
  #   xz_hsv           image from camera2 in HSV color format, where the camera angle is orthogonal to the xz plane
  #   x0               x coordinate of the base of the robot, where the origin is the top left of camera2's view
  #   y0               y coordinate of the base of the robot, where the origin is the top left of camera1's view
  #   yz0              z coordinate of the base of the robot, where the origin is the top left of camera1's view
  #   xz0              z coordinate of the base of the robot, where the origin is the top left of camera2's view
  #   lower_bound_val  lower bound of the Value in HSV to detect the color of joint
  #   upper_bound_val  upper bound of the Value in HSV to detect the color of joint
  # 
  # Returns None if the joint cannot be seen in either camera angle.
  def find_ball_3d_coordinates(self, yz_hsv, xz_hsv, x0, y0, yz0, xz0, lower_bound_val, upper_bound_val):
    yz_coords = self.find_ball_2d_coordinates(yz_hsv, lower_bound_val, upper_bound_val)
    xz_coords = self.find_ball_2d_coordinates(xz_hsv, lower_bound_val, upper_bound_val)

    if yz_coords is None or xz_coords is None:
      return None # Joint cannot be seen

    x = x0 - xz_coords[0]
    y = y0 - yz_coords[0]
    yz = yz0 - yz_coords[1]
    xz = xz0 - xz_coords[1]
    z = int((yz+xz)/2)
    return np.array([x, y, z])



  # Process images and determine [x,y,z] for base and each joint
  # Returns a dictionary containing coordinates 3d coordinates for each joint
  def process_images(self, yz_camera_image, xz_camera_image):
    yz_hsv = cv2.cvtColor(yz_camera_image, cv2.COLOR_BGR2HSV)
    xz_hsv = cv2.cvtColor(xz_camera_image, cv2.COLOR_BGR2HSV)

    # Base
    yellow_yz_coords = self.find_ball_2d_coordinates(yz_hsv, 25, 35)
    yellow_xz_coords = self.find_ball_2d_coordinates(xz_hsv, 25, 35)

    if yellow_yz_coords is None or yellow_xz_coords is None:
      print("Cannot see base yellow joint")
      return

    x0 = yellow_xz_coords[0]
    y0 = yellow_yz_coords[0]
    yz0 = yellow_yz_coords[1]
    xz0 = yellow_xz_coords[1]
    z0 = int((yz0+xz0)/2)

    coordinates = { 'yellow': [0, 0, 0] }
    joints = [['blue', 100, 130], ['green', 40, 70], ['red', 0, 10]]

    for joint in joints:
      coords = self.find_ball_3d_coordinates(yz_hsv, xz_hsv, x0, y0, yz0, xz0, joint[1], joint[2])

      if coords is None:
        print(f"cannot see {joint[0]} joint")
      else:

        if joint[0] == 'blue': # Blue joint x,y should be always equal to base x,y
          coords[0] = 0
          coords[1] = 0

        yz_camera_image = self.add_circle_to_image(yz_camera_image, [y0-coords[1], z0-coords[2]])
        xz_camera_image = self.add_circle_to_image(xz_camera_image, [x0-coords[0], z0-coords[2]])

        coordinates[joint[0]] = coords

    # Circles
    yz_camera_image = self.add_circle_to_image(yz_camera_image, [y0, z0])
    xz_camera_image = self.add_circle_to_image(xz_camera_image, [x0, z0])

    cv2.imshow('yz camera 1', yz_camera_image)
    cv2.imshow('xz camera 2', xz_camera_image)
    cv2.waitKey(100)

    return coordinates



  # Receive the camera data from both camera1 and camera2 and process
  def camera_sub(self, yz_camera_data, xz_camera_data):
    try:
      yz_camera_image = self.bridge.imgmsg_to_cv2(yz_camera_data, "bgr8")
      xz_camera_image = self.bridge.imgmsg_to_cv2(xz_camera_data, "bgr8")
      coords = self.process_images(yz_camera_image, xz_camera_image)

      joints = { "blue": self.blue_coord_pub, "green": self.green_coord_pub, "red": self.red_coord_pub }
      for key, publisher in joints.items():
        if key in coords:
          message = Float64MultiArray()
          message.data = coords[key]
          publisher.publish(message)

    except CvBridgeError as e:
      print(e)
      

  # --- Helper methods to test ---

  def add_circle_to_image(self, image, coords):
    if coords is None:
      return image
    else:
      return cv2.circle(image, (coords[0], coords[1]), 3, (0, 0, 0), -1)

  def joint3callback(self, data):
    self.joint3actual = data.data

  # ------------------------------




def main(args):
  icp = image_coordinate_processor()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


