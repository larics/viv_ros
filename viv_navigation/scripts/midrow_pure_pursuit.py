#!/usr/bin/env python3
import rospy
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseActionResult
import tf2_ros
import tf2_geometry_msgs

import tf

import numpy as np

from copy import deepcopy

def get_quaternion_from_euler(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  return [qw, qx, qy, qz]

class midrow_pure_pursuit:
  def __init__(self):
    rospy.init_node('midrow_pure_pursuit')

    self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    self.rate = rospy.Rate(30)
    self.KP_linear = 0.5
    self.KP_angular = 1
    rospy.Subscriber("/pure_pursuit_point", PointStamped, self.midline_callback)
    rospy.Subscriber("/this_row_enter_point", PointStamped, self.this_row_enter_callback)
    rospy.Subscriber("/right_row_enter_point", PointStamped, self.right_row_enter_callback)
    rospy.Subscriber("/left_row_enter_point", PointStamped, self.left_row_enter_callback)
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.move_base_result_callback)
    self.cmd_vel_topic = "/viv/spraying_mpc/cmd_vel"
    #self.cmd_vel_topic = "/viv/viv_velocity_controller/cmd_vel"
    self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
    self.front_distance_to_end = 9999
    self.min_front_vel = 0.1
    self.max_front_vel = 0.25

    self.row_width = 1.7

    self.move_base_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    
    self.reached_end_of_row = False

    self.state = "MIDROW_NAV"
    self.pure_pursuit_point = Point()
    self.right_row_enter_point = Point()
    self.this_row_enter_point = Point()
    self.left_row_enter_point = Point()
    self.new_pure_pursuit_point_rec = False
    self.move_base_result = False

    self.direction = "RIGHT"
    self.goal_sent = False

    self.current_nav_goal = 0
    self.goal_num = 2
    self.entered_cart_nav_first_time = True

    self.goal_list = []

  def midline_callback(self, msg):
    self.pure_pursuit_point = msg.point
    self.new_pure_pursuit_point_rec = True
    
  def this_row_enter_callback(self, msg):
    self.front_distance_to_end = float(msg.point.x)
    self.this_row_enter_point = msg.point
  def right_row_enter_callback(self, msg):
    self.right_row_enter_point = msg.point
  def left_row_enter_callback(self, msg):
    self.left_row_enter_point = msg.point
  def move_base_result_callback(self, msg):
    if (msg.status.text == "Goal reached."):
      self.move_base_result = True

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

    if(cmd_vel.linear.x > self.max_front_vel):
      cmd_vel.linear.x = self.max_front_vel

    if(cmd_vel.linear.x > self.min_front_vel):
      self.cmd_vel_pub.publish(cmd_vel)
    elif(self.front_distance_to_end < 1.0):
      self.state = "CARTOGRAPHER_NAV"

  def cartographer_nav_loop(self):
    transform = self.tf_buffer.lookup_transform("map",
                                                # source frame:
                                                "rslidar",
                                                # get the tf at the time the pose was valid
                                                rospy.Time.now(),
                                                # wait for at most 1 second for transform, otherwise throw
                                                rospy.Duration(1.0))
    if(self.entered_cart_nav_first_time):
      print("tu smo")
      self.row_width = abs(self.right_row_enter_point.y - self.this_row_enter_point.y)
      del self.goal_list[:]
      print(self.goal_list)
      msg = PoseStamped()
      msg.header.frame_id = "rslidar"
      if(self.direction == "RIGHT"):
        msg.pose.position.x = self.this_row_enter_point.x + 3
        msg.pose.position.y = self.this_row_enter_point.y + 2
        msg.pose.position.z = self.right_row_enter_point.z
        desired_quaternion = get_quaternion_from_euler(0, 0, 45 * 3.14159 / 180.0)
        msg.pose.orientation.w = desired_quaternion[0]
        msg.pose.orientation.x = desired_quaternion[1]
        msg.pose.orientation.y = desired_quaternion[2]
        msg.pose.orientation.z = desired_quaternion[3]
        transformed_pose = tf2_geometry_msgs.do_transform_pose(msg, transform)
        msg = transformed_pose
        self.flattenMsg(msg)
        self.goal_list.append(deepcopy(msg))
        msg.pose.position.x = self.this_row_enter_point.x + 2
        msg.pose.position.y = self.this_row_enter_point.y + 1
        msg.pose.position.z = self.right_row_enter_point.z
        desired_quaternion = get_quaternion_from_euler(0, 0, 90 * 3.14159 / 180.0)
        msg.pose.orientation.w = desired_quaternion[0]
        msg.pose.orientation.x = desired_quaternion[1]
        msg.pose.orientation.y = desired_quaternion[2]
        msg.pose.orientation.z = desired_quaternion[3]
        transformed_pose = tf2_geometry_msgs.do_transform_pose(msg, transform)
        msg = transformed_pose
        self.flattenMsg(msg)
        self.goal_list.append(deepcopy(msg))
        msg.pose.position.x = self.this_row_enter_point.x + 3
        msg.pose.position.y = self.this_row_enter_point.y + 0
        msg.pose.position.z = self.right_row_enter_point.z
        desired_quaternion = get_quaternion_from_euler(0, 0, (180 - 45) * 3.14159 / 180.0)
        msg.pose.orientation.w = desired_quaternion[0]
        msg.pose.orientation.x = desired_quaternion[1]
        msg.pose.orientation.y = desired_quaternion[2]
        msg.pose.orientation.z = desired_quaternion[3]
        transformed_pose = tf2_geometry_msgs.do_transform_pose(msg, transform)
        msg = transformed_pose
        self.flattenMsg(msg)
        self.goal_list.append(deepcopy(msg))
        msg.pose.position.x = self.this_row_enter_point.x + 1
        msg.pose.position.y = self.this_row_enter_point.y + self.row_width
        msg.pose.position.z = self.this_row_enter_point.z
        desired_quaternion = get_quaternion_from_euler(0, 0, 180 * 3.14159 / 180.0)
        msg.pose.orientation.w = desired_quaternion[0]
        msg.pose.orientation.x = desired_quaternion[1]
        msg.pose.orientation.y = desired_quaternion[2]
        msg.pose.orientation.z = desired_quaternion[3]
      if(self.direction == "LEFT"):
        print("Left not implemented!")
        """msg.pose.position.x = self.left_row_enter_point.x + 1
        msg.pose.position.y = self.left_row_enter_point.y + 1
        msg.pose.position.z = self.left_row_enter_point.z
        desired_quaternion = get_quaternion_from_euler(0, 0, -90 * 3.14159 / 180.0)
        msg.pose.orientation.w = desired_quaternion[0]
        msg.pose.orientation.x = desired_quaternion[1]
        msg.pose.orientation.y = desired_quaternion[2]
        msg.pose.orientation.z = desired_quaternion[3]
        transformed_pose = tf2_geometry_msgs.do_transform_pose(msg, transform)
        msg = transformed_pose
        self.flattenMsg(msg)
        self.goal_list.append(deepcopy(msg))
        msg.pose.position.x = self.left_row_enter_point.x + 0
        msg.pose.position.y = self.left_row_enter_point.y
        msg.pose.position.z = self.left_row_enter_point.z
        desired_quaternion = get_quaternion_from_euler(0, 0, 180 * 3.14159 / 180.0)
        msg.pose.orientation.w = desired_quaternion[0]
        msg.pose.orientation.x = desired_quaternion[1]
        msg.pose.orientation.y = desired_quaternion[2]
        msg.pose.orientation.z = desired_quaternion[3]"""
      
      transformed_pose = tf2_geometry_msgs.do_transform_pose(msg, transform)
      msg = transformed_pose
      self.flattenMsg(msg)
      self.goal_list.append(deepcopy(msg))
      self.entered_cart_nav_first_time = False

      print(self.goal_list)
      self.goal_num = len(self.goal_list)
    

    if(not self.goal_sent):
      self.goal_list[self.current_nav_goal].header.stamp = rospy.Time.now()
      msg = self.goal_list[self.current_nav_goal]
      self.move_base_goal_pub.publish(msg)
      self.goal_sent = True

    if(self.move_base_result):
      self.move_base_result = False
      self.goal_sent = False
      self.current_nav_goal = self.current_nav_goal + 1
      if(self.current_nav_goal == self.goal_num):
        if(self.direction == "RIGHT"):
          self.direction = "LEFT"
        elif(self.direction == "LEFT"):
          self.direction = "RIGHT"
        self.state = "MIDROW_NAV"
        self.entered_cart_nav_first_time = True
        self.current_nav_goal = 0

  def flattenMsg(self, msg):
    msg.pose.position.z = 0.0
    quat_msg = msg.pose.orientation
      
    quaternion = (quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w)
    [r, p, y] = tf.transformations.euler_from_quaternion(quaternion)
      
    r = 0
    p = 0
    quaternion = tf.transformations.quaternion_from_euler(r, p, y)
    quat_msg.x = quaternion[0]
    quat_msg.y = quaternion[1]
    quat_msg.z = quaternion[2]
    quat_msg.w = quaternion[3]
    msg.pose.orientation = quat_msg

  def loop(self):
    while not rospy.is_shutdown():
      if(self.state == "MIDROW_NAV"):
        self.midrow_nav_loop()
      if(self.state == "CARTOGRAPHER_NAV"):
        self.cartographer_nav_loop()
      if(self.state == "IDLE"):
        print("in idle..")
        time.sleep(1)

      self.rate.sleep()
  
if __name__ == '__main__':
  node = midrow_pure_pursuit()
  node.loop()