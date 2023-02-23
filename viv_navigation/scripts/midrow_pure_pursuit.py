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
    self.cmd_vel_pub = rospy.Publisher("/viv/spraying_mpc/cmd_vel", Twist, queue_size=1)
  def midline_callback(self, msg):
    cmd_vel = Twist()
    cmd_vel.linear.x = self.KP_linear * float(msg.point.x)
    cmd_vel.angular.z = self.KP_angular * msg.point.y

    self.cmd_vel_pub.publish(cmd_vel)


  def loop(self):
    while not rospy.is_shutdown():
      self.rate.sleep()
  
if __name__ == '__main__':
  node = midrow_pure_pursuit()
  node.loop()