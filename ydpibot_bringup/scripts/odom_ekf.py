#!/usr/bin/env python3

import dataclasses
import rospy
import math
import tf
from math import pi, cos, sin
from time import time
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from threading import Thread

class Node:
    def __init__(self) -> None:
        """
        """
        rospy.init_node("OdometryEKF_correction")

        self.odom = PoseWithCovarianceStamped()
        self.odom_publisher = rospy.Publisher("/odom", PoseWithCovarianceStamped, queue_size=5)

        self.transform = tf.TransformBroadcaster()

        rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.publish_odom)
        
        rospy.spin()
        
    def publish_odom(self,odom) -> None:
        """
        """
        self.odom.header.frame_id = "odom"
        self.odom.header.stamp = rospy.Time.now()

        self.odom.pose.pose.position.x = odom.pose.pose.position.x
        self.odom.pose.pose.position.y = odom.pose.pose.position.y

        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y

        yaw = euler_from_quaternion([odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w])[2]
        quat = quaternion_from_euler(0,0,yaw)

        self.odom.pose.pose.orientation.x = quat[0]
        self.odom.pose.pose.orientation.y = quat[1]
        self.odom.pose.pose.orientation.z = quat[2]
        self.odom.pose.pose.orientation.w = quat[3]

        self.odom_publisher.publish(self.odom)

        self.transform.sendTransform(
            (x,y,0),
            (quat[0],quat[1],quat[2],quat[3]),
            rospy.Time.now(),
            "/base_link",
            "/odom"
        )        

if __name__ == "__main__":
    Node()