#!/usr/bin/env python 
import rospy
import time

import tf

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Quaternion

from gazebo_msgs.srv import GetModelState

import numpy as np


class gazebo_odom_sim:
  def __init__(self):
    rospy.init_node('gazebo_odom_sim')
    self.rate = rospy.Rate(30)
    self.br = tf.TransformBroadcaster()

    self.model_name = "viv"
    self.rel_entity_name = "ground_plane"

  def loop(self):
    while not rospy.is_shutdown():
      try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
          
        resp_coordinates = get_model_state(self.model_name, self.rel_entity_name)
        self.br.sendTransform((resp_coordinates.pose.position.x, resp_coordinates.pose.position.y, resp_coordinates.pose.position.z),
                              (resp_coordinates.pose.orientation.x, resp_coordinates.pose.orientation.y, resp_coordinates.pose.orientation.z, resp_coordinates.pose.orientation.w),
                              rospy.Time.now(),
                              "viv_base",
                              "odom")
      except rospy.ServiceException as e:
        rospy.loginfo("Get Model State service call failed:  {0}".format(e))

      self.rate.sleep()
  
if __name__ == '__main__':
  node = gazebo_odom_sim()
  node.loop()