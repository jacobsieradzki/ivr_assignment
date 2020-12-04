#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import math
import message_filters
import itertools
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

    self.previousResult = { "yellow": [0,0,0,0,0], "blue": [0,0,0,0,0], "green": [0,0,0,0,0], "red": [0,0,0,0,0] }
    self.blue_coord_pub = rospy.Publisher("/jake/vision_coordinates/blue", Float64MultiArray, queue_size=10)
    self.green_coord_pub = rospy.Publisher("/jake/vision_coordinates/green", Float64MultiArray, queue_size=10)
    self.red_coord_pub = rospy.Publisher("/jake/vision_coordinates/red", Float64MultiArray, queue_size=10)

    self.bridge = CvBridge()



  def find_ball_2d_coordinates(self, hsv, lower_bound_val, upper_bound_val):
    lower = np.array([lower_bound_val, 50, 50])
    upper = np.array([upper_bound_val, 255, 255])

    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((5, 5), np.uint8)
    dilated_mask = cv2.dilate(mask, kernel, iterations=3)
    contours, hierarchy = cv2.findContours(dilated_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
    return np.array([x, y, z, yz, xz]) / 22.5


  def generate_coordinates(self, key, yz, xz):
    return np.array([xz[key][0], yz[key][0], int((yz[key][1]+xz[key][1])/2), yz[key][1], xz[key][1]])


  def process_3d_image(self, yz_hsv, xz_hsv):
    yz_coords = self.process_2d_image(yz_hsv, 'yz')
    xz_coords = self.process_2d_image(xz_hsv, 'xz')

    if yz_coords is None or xz_coords is None:
      return None

    yellow = self.generate_coordinates('yellow', yz_coords, xz_coords)
    blue = yellow - self.generate_coordinates('blue', yz_coords, xz_coords)
    green = yellow - self.generate_coordinates('green', yz_coords, xz_coords)
    red = yellow - self.generate_coordinates('red', yz_coords, xz_coords)

    return [yellow, blue, green, red]


  def process_2d_image(self, hsv, window):
    lower = np.array([0, 0, 0])
    upper = np.array([255, 255, 50])

    mask = cv2.inRange(hsv, lower, upper)

    detected = []
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=0.9, minDist=20, minRadius=5, maxRadius=15, param1=100, param2=6)
    if circles is not None:
      circles = np.round(circles[0, :]).astype("int")
      if len(circles) == 4: detected = circles
    
    coordinates = {}
    remaining = []

    for (x, y, r) in detected:
      cv2.circle(hsv, (x, y), r, (128, 255, 128), 1)
      if "yellow" not in coordinates and x >= 395 and x <= 405 and y >= 530 and y <= 540:
        coordinates["yellow"] = [x, y]
        cv2.putText(hsv, "Y", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)
      elif "blue" not in coordinates and x >= 390 and x <= 410 and y >= 460 and y <= 480:
        coordinates["blue"] = [x, y]
        cv2.putText(hsv, "B", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)
      else:
        remaining.append((x, y, r))

    if "blue" in coordinates:
      blue = coordinates["blue"]
      for (x, y, r) in remaining:
        mx = int((x+blue[0])/2)
        my = int((y+blue[1])/2)
        if mask[my][mx] == 255:
          coordinates["green"] = [x, y]
          cv2.putText(hsv, "G", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)
        else:
          coordinates["red"] = [x, y]
          cv2.putText(hsv, "R", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)

    cv2.imshow(window, hsv)
    if len(coordinates) == 4:
      return coordinates
    else:
      return None


  # Process images and determine [x,y,z,yz,xz] for base and each joint
  # Returns a dictionary containing coordinates 3d coordinates for each joint
  def process_images(self, yz_camera_image, xz_camera_image):
    yz_hsv = cv2.cvtColor(yz_camera_image, cv2.COLOR_BGR2HSV)
    xz_hsv = cv2.cvtColor(xz_camera_image, cv2.COLOR_BGR2HSV)
    self.process_3d_image(yz_hsv, xz_hsv)


    

    # cv2.imshow('Camera 1 (yz)', yz_camera_image)
    # cv2.imshow('Camera 2 (xz)', xz_camera_image)
    cv2.waitKey(100)

    # return coordinates


  # Receive the camera data from both camera1 and camera2 and process
  def camera_sub(self, yz_camera_data, xz_camera_data):
    try:
      yz_camera_image = self.bridge.imgmsg_to_cv2(yz_camera_data, "bgr8")
      xz_camera_image = self.bridge.imgmsg_to_cv2(xz_camera_data, "bgr8")
      coords = self.process_images(yz_camera_image, xz_camera_image)

    except CvBridgeError as e:
      print(e)
      

  # --- Helper methods to test ---

  def add_circle_to_image(self, image, coords, white):
    return image
    if coords is None:
      return image
    else:
      if white:
        return cv2.circle(image, (coords[0], coords[1]), 3, (255, 255, 255), -1)
      else:
        return cv2.circle(image, (coords[0], coords[1]), 3, (0, 0, 0), -1)

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


