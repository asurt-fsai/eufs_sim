#!/usr/bin/env python3

"""velocity_localizer.py

This node transforms the ground truth state (Odometry message) from the global ie. "odom" frame to local ie. "base_footprint" twist message.
Created on Sat Jan 6 2021

@author: Ahmad Samy (imahmadsamy@outlook.com)
"""

import rospy
import tf
from tf.transformations import quaternion_matrix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Quaternion


class LocalVelocityPublisher:
  
  def __init__ (self):
    rospy.init_node("local_velocity_publisher")

    #CREATING INITIAL CONTAINER
    self.local_twist = TwistStamped()
    
    # FRAME AND TOPIC PARAMETERS
    self.global_topic = rospy.get_param(param_name='global_topic', default="/ground_truth/state_raw")
    self.local_topic = rospy.get_param(param_name='local_topic', default="/ground_truth/local_twist")
    self.local_frame = rospy.get_param(param_name='local_frame', default="base_footprint")
    
    # CREATING SUBSCRIBER & PUBLISHER
    self.listener = rospy.Subscriber(self.global_topic, Odometry, self.transform_callback, queue_size=1) 
    self.local_pub = rospy.Publisher(self.local_topic, TwistStamped, queue_size=1)

  def transform_callback(self, global_odom):
    # SETTING HEADER
    self.local_twist.header = global_odom.header
    self.local_twist.header.frame_id = self.local_frame
    
    quat_list = [global_odom.pose.pose.orientation.x, global_odom.pose.pose.orientation.y, global_odom.pose.pose.orientation.z, global_odom.pose.pose.orientation.w]

    R = quaternion_matrix(quat_list)
    global_linear_vel = [global_odom.twist.twist.linear.x, global_odom.twist.twist.linear.y, global_odom.twist.twist.linear.z]
    global_angular_vel = [global_odom.twist.twist.angular.x, global_odom.twist.twist.angular.y, global_odom.twist.twist.angular.z]

    self.local_twist.twist.linear.x = (R[:3,:3].T @ global_linear_vel)[0]
    self.local_twist.twist.linear.y = (R[:3,:3].T @ global_linear_vel)[1]
    self.local_twist.twist.linear.z = (R[:3,:3].T @ global_linear_vel)[2]
    self.local_twist.twist.angular.x = (R[:3,:3].T @ global_angular_vel)[0]
    self.local_twist.twist.angular.y = (R[:3,:3].T @ global_angular_vel)[1]
    self.local_twist.twist.angular.z = (R[:3,:3].T @ global_angular_vel)[2]
    self.local_pub.publish(self.local_twist)



if __name__ == '__main__':
  local_velocity_pub = LocalVelocityPublisher()
  rospy.spin()