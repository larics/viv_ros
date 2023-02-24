#!/usr/bin/env python3  
import rospy
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Quaternion

import numpy as np

def get_quaternion_from_euler(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  return [qw, qx, qy, qz]

class midrow_pure_pursuit:
  def __init__(self):
    rospy.init_node('midrow_pure_pursuit')
    self.rate = rospy.Rate(30)
    self.KP_linear = 0.1
    self.KP_angular = 0.5
    rospy.Subscriber("/pure_pursuit_point", PointStamped, self.midline_callback)
    rospy.Subscriber("/this_row_enter_point", PointStamped, self.this_row_enter_callback)
    rospy.Subscriber("/right_row_enter_point", PointStamped, self.right_row_enter_callback)
    rospy.Subscriber("/left_row_enter_point", PointStamped, self.left_row_enter_callback)
    #self.cmd_vel_topic = "/viv/spraying_mpc/cmd_vel"
    self.cmd_vel_topic = "/viv/viv_velocity_controller/cmd_vel"
    self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
    self.front_distance_to_end = 9999
    self.min_front_vel = 0.05

    self.move_base_goal_pub = rospy.Publisher("/erl_husky/move_base_simple/goal", PoseStamped, queue_size=1)
    
    self.reached_end_of_row = False

    self.state = "MIDROW_NAV"
    self.pure_pursuit_point = Point()
    self.right_row_enter_point = Point()
    self.left_row_enter_point = Point()
    self.new_pure_pursuit_point_rec = False

    self.direction = "RIGHT"

  def midline_callback(self, msg):
    self.pure_pursuit_point = msg.point
    self.new_pure_pursuit_point_rec = True
    
  def this_row_enter_callback(self, msg):
    self.front_distance_to_end = float(msg.point.x)
  def right_row_enter_callback(self, msg):
    self.right_row_enter_point = msg.point
  def left_row_enter_callback(self, msg):
    self.left_row_enter_point = msg.point

  def midrow_nav_loop(self):
    if(not self.new_pure_pursuit_point_rec):
      return
    self.new_pure_pursuit_point_rec = False
    cmd_vel = Twist()
    if(self.pure_pursuit_point.x < self.front_distance_to_end):
      cmd_vel.linear.x = self.KP_linear * self.pure_pursuit_point.x
    else:
      cmd_vel.linear.x = self.KP_linear * self.front_distance_to_end
    cmd_vel.angular.z = self.KP_angular * self.pure_pursuit_point.y

    if(cmd_vel.linear.x > self.min_front_vel):
      self.cmd_vel_pub.publish(cmd_vel)
    elif(self.front_distance_to_end < 1.0):
      self.state = "CARTOGRAPHER_NAV"

  def cartographer_nav_loop(self):
    if(self.direction == "LEFT"):
      msg = PoseStamped()
      msg.header.frame_id = "rslidar"
      msg.header.stamp = rospy.Time.now()
      msg.pose.position.x = self.left_row_enter_point.x
      msg.pose.position.y = self.left_row_enter_point.y
      msg.pose.position.z = self.left_row_enter_point.z
      desired_quaternion = get_quaternion_from_euler(0, 0, 180 * 3.14159 / 180.0)
      msg.pose.orientation.w = desired_quaternion[0]
      msg.pose.orientation.x = desired_quaternion[1]
      msg.pose.orientation.y = desired_quaternion[2]
      msg.pose.orientation.z = desired_quaternion[3]
      self.move_base_goal_pub.publish(msg)
    if(self.direction == "RIGHT"):
      msg = PoseStamped()
      msg.header.frame_id = "rslidar"
      msg.header.stamp = rospy.Time.now()
      msg.pose.position.x = self.right_row_enter_point.x
      msg.pose.position.y = self.right_row_enter_point.y
      msg.pose.position.z = self.right_row_enter_point.z
      desired_quaternion = get_quaternion_from_euler(0, 0, 180 * 3.14159 / 180.0)
      msg.pose.orientation.w = desired_quaternion[0]
      msg.pose.orientation.x = desired_quaternion[1]
      msg.pose.orientation.y = desired_quaternion[2]
      msg.pose.orientation.z = desired_quaternion[3]
      self.move_base_goal_pub.publish(msg)

    time.sleep(4)
    self.state = "MIDROW_NAV"

  def loop(self):
    while not rospy.is_shutdown():
      if(self.state == "MIDROW_NAV"):
        self.midrow_nav_loop()
      if(self.state == "CARTOGRAPHER_NAV"):
        self.cartographer_nav_loop()

      self.rate.sleep()
  
if __name__ == '__main__':
  node = midrow_pure_pursuit()
  node.loop()