#!/usr/bin/env python3  
import rospy

import copy
# Because of transformations
import tf_conversions
import tf
import tf2_ros
import geometry_msgs.msg
from tf2_msgs.msg import TFMessage

import time

class tf_republisher_node:
  def __init__(self):
    rospy.init_node('move_base_tf_republisher')
    self.rate = rospy.Rate(10)

    self.listener = tf.TransformListener()
    self.broadcaster = tf2_ros.TransformBroadcaster()
    
  def republish_tf(self):
    trans_stamped_move_base = geometry_msgs.msg.TransformStamped()
    try:
      (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
      trans_stamped_move_base.transform.translation.x = trans[0]
      trans_stamped_move_base.transform.translation.y = trans[1]
      trans_stamped_move_base.transform.translation.z = 0.0
      trans_stamped_move_base.transform.rotation.x = rot[0]
      trans_stamped_move_base.transform.rotation.y = rot[1]
      trans_stamped_move_base.transform.rotation.z = rot[2]
      trans_stamped_move_base.transform.rotation.w = rot[3]
      
      quat_msg = trans_stamped_move_base.transform.rotation
      
      quaternion = (quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w)
      [r, p, y] = tf.transformations.euler_from_quaternion(quaternion)
      
      r = 0
      p = 0
      quaternion = tf.transformations.quaternion_from_euler(r, p, y)
      quat_msg.x = quaternion[0]
      quat_msg.y = quaternion[1]
      quat_msg.z = quaternion[2]
      quat_msg.w = quaternion[3]
      
      trans_stamped_move_base.transform.rotation = quat_msg

      trans_stamped_move_base.header.frame_id = 'map'
      trans_stamped_move_base.child_frame_id = 'move_base_link'
      trans_stamped_move_base.header.stamp = rospy.Time.now()

      self.broadcaster.sendTransform(trans_stamped_move_base)

    except:
      print ("fail!")
      return

  def loop(self):
    while not rospy.is_shutdown():
      self.republish_tf()
      self.rate.sleep()

if __name__ == '__main__':
  node = tf_republisher_node()
  node.loop()