#!/usr/bin/env python3  
import rospy

import copy
# Because of transformations
import tf_conversions
import tf
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan

import time

class scan_republisher_node:
  def __init__(self):
    rospy.init_node('move_base_scan_republisher')
    self.rate = rospy.Rate(10)

    rospy.Subscriber("/viv/scan", LaserScan, self.laserscan_callback)
    self.mb_scan_pub = rospy.Publisher("/viv/move_base_scan", LaserScan, queue_size=1)
    
  def laserscan_callback(self, msg):
    msg.header.frame_id = "move_base_link"
    self.mb_scan_pub.publish(msg)
    
if __name__ == '__main__':
  node = scan_republisher_node()
  rospy.spin()
