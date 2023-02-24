#!/usr/bin/env python3  
import rospy
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped

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
    
  def midline_callback(self, msg):
    cmd_vel = Twist()
    if(float(msg.point.x) < self.front_distance_to_end):
      cmd_vel.linear.x = self.KP_linear * float(msg.point.x)
    else:
      cmd_vel.linear.x = self.KP_linear * self.front_distance_to_end
    cmd_vel.angular.z = self.KP_angular * msg.point.y

    if(cmd_vel.linear.x > self.min_front_vel):
      self.cmd_vel_pub.publish(cmd_vel)

  def this_row_enter_callback(self, msg):
    self.front_distance_to_end = float(msg.point.x)

  def loop(self):
    while not rospy.is_shutdown():
      self.rate.sleep()
  
if __name__ == '__main__':
  node = midrow_pure_pursuit()
  node.loop()