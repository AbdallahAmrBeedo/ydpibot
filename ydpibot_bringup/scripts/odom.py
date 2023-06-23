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

@dataclasses.dataclass
class Param:
    """Tuning and utils params"""
    wheel_base = rospy.get_param("/robot/wheel_base")
    wheel_radius = rospy.get_param("/robot/wheel_radius")
    encoder_resolution = rospy.get_param("/robot/encoder_resolution")

@dataclasses.dataclass
class Time:
    """Time related variables for integration and differentiation"""

    t_prev = 0
    t_current = 0
    delta_t = 0

@dataclasses.dataclass
class Encoders:
    """The readings of the encoders ticks"""

    right = 0
    left = 0

@dataclasses.dataclass
class Measured:
    """The measured values from the encoders"""
    
    d_right = 0     # d_r = 2 * R * pi * right_ticks / resolution
    d_left = 0      # d_l = 2 * R * pi * left_ticks / resolution
    d_center = 0    # d_c = (d_r + d_l)/ 2
    yaw_prev = 0
    yaw = 0         # yaw = (d_r - d_l) / Wheel_base
    x = 0           # x = d_c * cos(yaw)
    y = 0           # y = d_c * sin(yaw) 
    v = 0           # v = delta(d_c) / delta_t
    w = 0           # w = delta(yaw) / delta_t

PARAM = Param()
t = Time()
current_ticks = Encoders()
prev_ticks = Encoders()
measurments = Measured()

class Node:
    def __init__(self) -> None:
        """
        """
        rospy.init_node("Wheel_Odometry")

        self.rate = rospy.Rate(30)
        self.odom = Odometry()
        self.odom_publisher = rospy.Publisher("/wheel_odom", Odometry, queue_size=5)

        transform = tf.TransformBroadcaster()

        for i in range(36):
            if i == 0 or i == 7 or i == 14:
                self.odom.pose.covariance[i] = 0.01
            elif i == 21 or i == 28 or i == 35:
                self.odom.pose.covariance[i] = 0.01
            else:
                self.odom.pose.covariance[i] = 0

        t.t_prev = time()
        
        Thread(target=self.subscribers).start()
        Thread(target=self.publish_odom).start()

    
    def subscribers(self):
        rospy.Subscriber("/right_ticks", Int16, self.right_distance)
        rospy.Subscriber("/left_ticks", Int16, self.left_distance)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.set_init_pose)
        rospy.spin()
        
    def right_distance(self, ticks) -> None:
        """
        """
        current_ticks.right = ticks.data
        self.odom.pose.covariance[21] += 0.01
        self.odom.pose.covariance[28] += 0.01


    def left_distance(self, ticks) -> None:
        """
        """
        current_ticks.left = ticks.data
        self.odom.pose.covariance[21] += 0.01
        self.odom.pose.covariance[35] += 0.01
            
    def set_init_pose(self,init) -> None:
        """
        """
        measurments.x  -= init.pose.pose.position.x
        measurments.y  -= init.pose.pose.position.y
        euler = euler_from_quaternion([init.pose.pose.orientation.x, init.pose.pose.orientation.y, init.pose.pose.orientation.z, init.pose.pose.orientation.w])
        measurments.yaw -= euler[2]
        self.odom.pose.covariance[21] = 0.01
        self.odom.pose.covariance[28] += 0.01
        self.odom.pose.covariance[35] = 0.01
        
    def calc_odom(self) -> None:
        """
        """
        t.t_current = time()
        t.delta_t = t.t_current - t.t_prev

        measurments.d_left = (current_ticks.left - prev_ticks.left) * 2 * pi * PARAM.wheel_radius / PARAM.encoder_resolution
        measurments.d_right = (current_ticks.right - prev_ticks.right) * 2 * pi * PARAM.wheel_radius / PARAM.encoder_resolution
        
        prev_ticks.left = current_ticks.left
        prev_ticks.right = current_ticks.right

        measurments.d_center = (measurments.d_left + measurments.d_right) / 2
        measurments.yaw += (measurments.d_right - measurments.d_left) / PARAM.wheel_base
        
        if measurments.yaw > pi:
            measurments.yaw -= 2*pi
        elif measurments.yaw < -pi:
            measurments.yaw += 2*pi
        
        measurments.x += measurments.d_center * cos(measurments.yaw)
        measurments.y += measurments.d_center * sin(measurments.yaw)

        measurments.v = measurments.d_center / t.delta_t
        measurments.w = (measurments.yaw - measurments.yaw_prev) / t.delta_t

        measurments.yaw_prev = measurments.yaw

        t.t_prev = t.t_current
    
    def publish_odom(self) -> None:
        """
        """
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"

        while not rospy.is_shutdown():

            self.calc_odom()

            self.odom.header.stamp = rospy.Time.now()
            self.odom.pose.pose.position.x = measurments.x
            self.odom.pose.pose.position.y = measurments.y
            self.odom.pose.pose.position.z = 0
            
            quat = quaternion_from_euler(0,0,measurments.yaw)
            self.odom.pose.pose.orientation.x = quat[0]
            self.odom.pose.pose.orientation.y = quat[1]
            self.odom.pose.pose.orientation.z = quat[2]
            self.odom.pose.pose.orientation.w = quat[3]

            self.odom.twist.twist.linear.x = measurments.v
            self.odom.twist.twist.angular.z = measurments.w
        
            self.odom_publisher.publish(self.odom)
            
            self.rate.sleep()

if __name__ == "__main__":
    Node()