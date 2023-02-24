#!/usr/bin/env python3  
import rospy
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped, PoseStamped, Point

class midrow_pure_pursuit:
  def __init__(self):
    rospy.init_node('midrow_pure_pursuit')
    self.rate = rospy.Rate(30)
    self.KP_linear = 0.1
    self.KP_angular = 0.5
    rospy.Subscriber("/pure_pursuit_point", PointStamped, self.midline_callback)
    rospy.Subscriber("/this_row_enter_point", PointStamped, self.this_row_enter_callback)
    #self.cmd_vel_topic = "/viv/spraying_mpc/cmd_vel"
    self.cmd_vel_topic = "/viv/viv_velocity_controller/cmd_vel"
    self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
    self.front_distance_to_end = 9999
    self.min_front_vel = 0.05

    self.move_base_goal_pub = rospy.Publisher("/erl_husky/move_base_simple/goal", PoseStamped, queue_size=1)
    
    self.reached_end_of_row = False

    self.state = "MIDROW_NAV"
    self.pure_pursuit_point = Point()
    self.new_pure_pursuit_point_rec = False

  def midline_callback(self, msg):
    self.pure_pursuit_point = msg.point
    self.new_pure_pursuit_point_rec = True
    
  def this_row_enter_callback(self, msg):
    self.front_distance_to_end = float(msg.point.x)

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
    print("Cartographer oh yeaaa!")

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