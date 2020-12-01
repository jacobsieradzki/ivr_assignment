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


class image_converter:

  def __init__(self):
    rospy.init_node('image_processing', anonymous=True)
    self.sync = message_filters.ApproximateTimeSynchronizer([
      message_filters.Subscriber("/camera1/robot/image_raw", Image),
      message_filters.Subscriber("/camera2/robot/image_raw", Image)
    ], 10, 0.1, allow_headerless=True)
    self.sync.registerCallback(self.camera_sub)

    self.joint2sub = rospy.Subscriber("/robot/joint2_position_controller/command", Float64, self.joint2callback)
    self.joint2actual = 0

    self.bridge = CvBridge()




  def joint2callback(self, data):
    self.joint2actual = data.data

  def process(self, image, window_name):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    combined = image

    yellow = self.find_ball_coordinates(image, hsv, 25, 35)
    green = self.find_ball_coordinates(image, hsv, 40, 70)
    blue = self.find_ball_coordinates(image, hsv, 100, 130)
    red = self.find_ball_coordinates(image, hsv, 0, 10)

    if yellow is None or green is None or blue is None or red is None:
      return


    # observed_link_1_distance = math.sqrt((blue[0]-yellow[0])**2 + (blue[1]-yellow[1])**2)
    # distance_scale = observed_link_1_distance / 2.5
    # print('observed_link_1_distance', window_name, observed_link_1_distance, distance_scale)

    # observed_link_3_distance = math.sqrt((blue[0]-green[0])**2 + (blue[1]-green[1])**2)

    # scaled_link_3_distance = observed_link_3_distance / distance_scale
    # print('scaled_link_3_distance', scaled_link_3_distance)
    if window_name == 'zy':
      # 0=y, 1=z
      o = green[0]-blue[0]
      a = green[1]-blue[1]
      joint_2_angle = 0
      if not a == 0:
        joint_2_angle = math.atan(-o/a)
      diff = abs(joint_2_angle - self.joint2actual)
      if diff > 0.1:
        print('joint_2_angle: diff > 0.1', joint_2_angle, self.joint2actual, diff)

    



    for coords in [yellow, green, blue, red]:
      if not coords is None:
        [cx, cy] = coords
        combined = cv2.circle(combined, (cx, cy), 3, (0, 0, 0), -1)

    cv2.imshow(window_name, combined)
    cv2.waitKey(100)


  def find_ball_coordinates(self, image, hsv, lower_color_bound, upper_color_bound):
    lower = np.array([lower_color_bound, 50, 50])
    upper = np.array([upper_color_bound, 255, 255])

    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(image, image, mask=mask)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
      M = cv2.moments(contours[0])
      if M['m00'] == 0:
        # Item is invisible
        return None
      else:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return [cx, cy]
    else:
      return None


  def zy_camera_callback(self, data):
    try:
      zy_camera_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.process(zy_camera_image, 'zy')

    except CvBridgeError as e:
      print(e)


  def zx_camera_callback(self, data):
    try:
      zx_camera_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      # self.process(zx_camera_image, 'zx')

    except CvBridgeError as e:
      print(e)


















  def find_ball_3d_coordinates(self, yz_image, yz_hsv, xz_image, xz_hsv, x0, y0, yz0, xz0, lower_bound, upper_bound):
    yz_coords = self.find_ball_coordinates(yz_image, yz_hsv, lower_bound, upper_bound)
    xz_coords = self.find_ball_coordinates(xz_image, xz_hsv, lower_bound, upper_bound)

    if yz_coords is None or xz_coords is None:
      return None

    x = x0 - xz_coords[0]
    y = y0 - yz_coords[0]
    yz = yz0 - yz_coords[1]
    xz = xz0 - xz_coords[1]
    z = int((yz+xz)/2)
    return [x, y, z]



  def add_circle_to_image(self, image, coords):
    if coords is None:
      return image
    else:
      return cv2.circle(image, (coords[0], coords[1]), 3, (0, 0, 0), -1)



  def print_coords(self, name, x, y, yz, xz):
    z = np.mean([yz, xz])
    print(f'{name} -> x={x}, y={y}, yz={yz}, xz={xz}, z={z}')



  def process_images(self, yz_camera_image, xz_camera_image):
    yz_hsv = cv2.cvtColor(yz_camera_image, cv2.COLOR_BGR2HSV)
    xz_hsv = cv2.cvtColor(xz_camera_image, cv2.COLOR_BGR2HSV)

    # Yellow
    yellow_yz_coords = self.find_ball_coordinates(yz_camera_image, yz_hsv, 25, 35)
    yellow_xz_coords = self.find_ball_coordinates(xz_camera_image, xz_hsv, 25, 35)

    if yellow_yz_coords is None or yellow_xz_coords is None:
      print("Cannot see base yellow joint")
      return

    x0 = yellow_xz_coords[0]
    y0 = yellow_yz_coords[0]
    yz0 = yellow_yz_coords[1]
    xz0 = yellow_xz_coords[1]
    z0 = int((yz0+xz0)/2)

    joints = [['blue', 100, 130], ['green', 40, 70], ['red', 0, 10]]
    for joint in joints:
      coords = self.find_ball_3d_coordinates(yz_camera_image, yz_hsv, xz_camera_image, xz_hsv, x0, y0, yz0, xz0, joint[1], joint[2])

      if coords is None:
        print(f"cannot see {joint[0]} joint")
      else:
        if joint[0] == 'blue':
          coords[1] = 0

        # print(joint[0], coords)

        yz_camera_image = self.add_circle_to_image(yz_camera_image, [y0-coords[1], z0-coords[2]])
        xz_camera_image = self.add_circle_to_image(xz_camera_image, [x0-coords[0], z0-coords[2]])

    # Circles
    yz_camera_image = self.add_circle_to_image(yz_camera_image, [y0, z0])
    xz_camera_image = self.add_circle_to_image(xz_camera_image, [x0, z0])

    cv2.imshow('yz camera 1', yz_camera_image)
    cv2.imshow('xz camera 2', xz_camera_image)
    cv2.waitKey(100)




  def camera_sub(self, yz_camera_data, xz_camera_data):
    try:
      yz_camera_image = self.bridge.imgmsg_to_cv2(yz_camera_data, "bgr8")
      xz_camera_image = self.bridge.imgmsg_to_cv2(xz_camera_data, "bgr8")
      self.process_images(yz_camera_image, xz_camera_image)

    except CvBridgeError as e:
      print(e)
      











  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)


    # Publish the results
    # try: 
      # self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    # except CvBridgeError as e:
      # print(e)



def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


