#!/usr/bin/env python3

"""
This ROS node implements a PID controller for a differential drive robot's motor speeds using IMU data and desired velocities. 

The node subscribes to the "imu" topic for orientation and angular velocity measurements and the "cmd_vel" topic for desired 
linear and angular velocities. It publishes the computed PWM values for each wheel on the "motor_speeds" topic as a multi-int array.

The PID controller is used to regulate the motor speeds and maintain the robot's orientation. The motor speeds are adjusted based 
on the error between the desired orientation and the actual orientation measured by the IMU, as well as the error between the 
desired and actual linear and angular velocities. The controller computes the proportional, integral, and derivative terms of the 
error and sums them to produce the output PWM values.

The robot has two wheels that are independently controlled and that the desired orientation is given as a euler angle.

Usage:
    $ rosrun ydpibot_bringup diff_drive_kinematic.py

Subscribed Topics:
    /cmd_vel (geometry_msgs/Twist)
    /imu (sensor_msgs/Imu)

Published Topics:
    /motor_speeds (std_msgs/Int16MultiArray)

Parameters:
    ~kp_x (float, default: 1.0): proportional gain of the PID controller for linear motion
    ~ki_x (float, default: 0.0):integral gain of the PID controller for linear motion
    ~kd_x (float, default: 0.0): derivative gain of the PID controller for linear motion
    ~kp_w (float, default: 1.0): proportional gain of the PID controller for angular motion
    ~ki_w (float, default: 0.0):integral gain of the PID controller for angular motion
    ~kd_w (float, default: 0.0): derivative gain of the PID controller for angular motion
    ~wheelbase (float, default: 0.15): distance between the two wheels
    ~wheel_radius (float, default: 0.0325): radius of the robot's wheels
    ~max_speed (int, default: 255): maximum PWM value for the motors
    ~min_speed (int, default: 0): minimum PWM value for the motors
    ~max_vx (float, default: 0.5): maximum linear velocity for the robot
    ~min_vx (float, default: -0.5): minimum linear velocity for the robot
    ~max_wz (float, default: 1): maximum angular velocity for the robot
    ~min_wz (float, default: -1): minimum angular velocity for the robot

Contributors:
    - Abdallah Amr (Egypt Japan University of Science and Technology)
    - Mostafa Osama (Egypt Japan University of Science and Technology)
    - Tarek Shohdy (Egypt Japan University of Science and Technology)
    - Yomna Omar (Egypt Japan University of Science and Technology)
"""

import dataclasses
import rospy
import math
from time import time
from typing import List
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from dynamic_reconfigure.server import Server
from ydpibot_bringup.cfg import pidConfig, robotConfig

@dataclasses.dataclass
class ErrorVal:
    """PID error terms"""

    e_sum: float = 0
    d_error: float = 0
    current_error: float = 0
    prev_error: float = 0

class PID:
    """PID controller Class"""

    def __init__(
        self,
        k_proportaiol: float,
        k_integral: float,
        k_derivative: float,
        windup_val: float,
    ) -> None:
        """Creates a PID controller using provided PID parameters

        Args:
            k_proportaiol (float): the proportional error constant of the desired PID
            k_integral (float): the Integral error constant of the desired PID
            k_derivative (float): the derivative error constant of the desired PID
        """
        self.k_proportaiol = k_proportaiol
        self.k_integral = k_integral
        self.k_derivative = k_derivative
        self.pid: float = 0
        self.error = ErrorVal()
        self.windup_val = windup_val

    def compute(self, ref: float, measured: float) -> float:
        """Computes the PID value based on the given reference and measured output values

        Args:
            ref (float): reference signal that we desire to track
            measured (float): actual measured output of the signal

        Returns:
            float: PID value
        """
        self.error.current_error = ref - measured
        self.error.e_sum += self.error.current_error
        self.error.d_error = self.error.current_error - self.error.prev_error
        self.pid = (
            self.k_proportaiol * self.error.current_error
            + self.k_integral * self.error.e_sum
            + self.k_derivative * self.error.d_error
        )
        self.error.prev_error = self.error.current_error
        if self.error.current_error <= self.windup_val:
            self.error.e_sum = 0
        return self.pid

    def set_pid(
        self, k_proportaiol: float, k_integral: float, k_derivative: float
    ) -> None:
        """Sets the PID controller constants

        Args:
            k_proportaiol (float): the proportional error constant of the desired PID
            k_integral (float): the Integral error constant of the desired PID
            k_derivative (float): the derivative error constant of the desired PID
        """
        self.k_proportaiol = k_proportaiol
        self.k_integral = k_integral
        self.k_derivative = k_derivative

@dataclasses.dataclass
class Param:
    """Tuning and utils params"""

    kp_x = rospy.get_param("/pid/x/kp")
    ki_x = rospy.get_param("/pid/x/ki")
    kd_x = rospy.get_param("/pid/x/kd")
    windup_val_x = rospy.get_param("/pid/x/windup")
    
    kp_w = rospy.get_param("/pid/w/kp")
    ki_w = rospy.get_param("/pid/w/ki")
    kd_w = rospy.get_param("/pid/w/kd")
    windup_val_w = rospy.get_param("/pid/w/windup")

    wheel_base = rospy.get_param("/robot/wheel_base")
    wheel_radius = rospy.get_param("/robot/wheel_radius")

    max_speed = rospy.get_param("/robot/max_speed")
    min_speed = rospy.get_param("/robot/min_speed")
    
    max_vx = rospy.get_param("/robot/max_vx")
    min_vx = rospy.get_param("/robot/min_vx")

    max_wz = rospy.get_param("/robot/max_wz")
    min_wz = rospy.get_param("/robot/min_wz")

@dataclasses.dataclass
class Measured:
    """Sensor measurments"""

    v_x = 0
    w_z = 0
    ax_prev = 0
    ax_current = 0

@dataclasses.dataclass
class Reference:
    """Reference values"""

    v_x = 0
    w_z = 0

@dataclasses.dataclass
class Time:
    """Time related variables for integration and differentiation"""

    t_prev = 0
    t_current = 0
    delta_t = 0

def map_from_to(
    num: float, in_min: float, in_max: float, out_min: float, out_max: float
) -> float:
    """Mapping function

    Args:
        num (float): Value to be mapped
        in_min (float): Minimum value of input
        in_max (float): Maximum value of input
        out_min (float): Minimum value of ouput
        out_max (float): Maximum value of ouput

    Returns:
        float: Mapped value
    """
    return (
        float(num - in_min) / float(in_max - in_min) * (out_max - out_min)
    ) + out_min

PARAM = Param()
actual = Measured()
ref = Reference()
t = Time()

pid_vx = PID(
    k_proportaiol=PARAM.kp_x,
    k_derivative=PARAM.kd_x,
    k_integral=PARAM.ki_x,
    windup_val=PARAM.windup_val_x,
)
pid_wz = PID(
    k_proportaiol=PARAM.kp_w,
    k_derivative=PARAM.kd_w,
    k_integral=PARAM.ki_w,
    windup_val=PARAM.windup_val_w,
)

class Robot:
    """ Robot Class to solve the kinematic model"""

    def __init__(self) -> None:
        """Intialize the robot parameters for the kinematic model"""
        self.wheel_radius = PARAM.wheel_radius
        self.wheel_base = PARAM.wheel_base
    
    def kinematic(self, vx: float, wz: float) -> List:
        """_summary_

        Args:
            vx (float): value for x linear velocity
            wz (float): value for w angular velocity

        Returns:
            List: List of two angular velocities for the motors (phi_r, phi_l)
        """
        phi_r = (2 * vx + wz * self.wheel_base) / self.wheel_radius
        phi_l = (2 * vx - wz * self.wheel_base) / self.wheel_radius
        return [phi_r, phi_l]
    
    def find_phi_boudary_values(self, param: PARAM) -> List:
        """Estimate the motors boundary value solely based on max allowed velocity in each axis

        Args:
            param (PARAM): PARAM data class

        Returns:
            List: Boundary values for the motors actuation (max_phi, min_phi)
        """
        max_phi = (2 * param.max_vx + param.max_wz * self.wheel_base) / self.wheel_radius
        min_phi = (2 * param.min_vx + param.min_wz * self.wheel_base) / self.wheel_radius
        return [max_phi, min_phi]
    
    def sturate(self,
                phi_r: float,
                phi_l: float,
                max_phi: float,
                min_phi: float
                ) -> List:
            """_summary_

            Args:
                phi_r (float): Unsaturated right motor value
                phi_l (float): Unsaturated left motor value
                min_phi (float): Reverse saturation value
                max_phi (float): Forward saturation value

            Returns:
                List: Saturated list of motors values (phi_r, phi_l)
            """
            phi_r = max(phi_r, min_phi)
            phi_r = min(phi_r, max_phi)
            phi_l = max(phi_l, min_phi)
            phi_l = min(phi_l, max_phi)
            return [phi_r, phi_l]
         

class Node:
    def __init__(self) -> None:
        """Initialize node which subscribes to /cmd_vel topic and the /imu and publishes on the /motor_speeds topic"""
        rospy.init_node("diff_drive_kinematic")

        rospy.on_shutdown(self.stopAll)
        
        self.robot = Robot()

        srvpid = Server(pidConfig, self.set_pid_param_callback, "pid")
        srvrobot = Server(robotConfig, self.set_robot_param_callback, "robot")
        
        self.motors = rospy.Publisher("/motor_speeds", Int16MultiArray, queue_size=10)
        self.motor_speeds = Int16MultiArray()

        t.t_prev = time()

        rospy.Subscriber("/cmd_vel", Twist, self.cmdvelCb)
        rospy.Subscriber("/imu", Imu, self.imuCb)
        rospy.spin()
    
    def set_pid_param_callback(self, config, level):
        """
        Dynamic reconfiguration of PID parameters to make the tuning much easier
        
        Args:
            config: the list of the paramter conigured with the new values coming from the server
            level: indication if the values are not changed yet
        """
        if not level:
            config['kp_x'] = PARAM.kp_x
            config['ki_x'] = PARAM.ki_x
            config['kd_x'] = PARAM.kd_x
            config['kp_w'] = PARAM.kp_w
            config['ki_w'] = PARAM.ki_w
            config['kd_w'] = PARAM.kd_w
            return config
        PARAM.kp_x = config['kp_x']
        PARAM.ki_x = config['ki_x']
        PARAM.kd_x = config['kd_x']
        PARAM.kp_w = config['kp_w']
        PARAM.ki_w = config['ki_w']
        PARAM.kd_w = config['kd_w']
        return config

    def set_robot_param_callback(self, config, level):
        """
        Dynamic reconfiguration of robot parameters to make tuning much easier
        
        Args:
            config: the list of the paramter conigured with the new values coming from the server
            level: indication if the values are not changed yet
        """
        if not level:
            config['max_speed'] = PARAM.max_speed
            config['min_speed'] = PARAM.min_speed
            config['max_vx'] = PARAM.max_vx
            config['min_vx'] = PARAM.min_vx
            config['max_wz'] = PARAM.max_wz
            config['min_wz'] = PARAM.min_wz
            pid_wz.set_pid(PARAM.kp_w, PARAM.ki_w, PARAM.kd_w)
            return config
        PARAM.max_speed = config['max_speed']
        PARAM.min_speed = config['min_speed']
        PARAM.max_vx =  config['max_vx']
        PARAM.min_vx = config['min_vx']
        PARAM.max_wz = config['max_wz']
        PARAM.min_wz = config['min_wz']
        pid_wz.set_pid(PARAM.kp_w, PARAM.ki_w, PARAM.kd_w)
        return config

    def cmdvelCb(self,cmd) -> None:
        """
        Callback of the subiscribed topic /cmd_vel to set reference values

        Args:
            cmd (Twist): command velocity sent by teleop node or any other node that publish to the same topic
        """
        ref.v_x = cmd.linear.x
        ref.w_z = cmd.angular.z

    def imuCb(self,imu) -> None:
        """recieves the imu readings from the MPU6050 and compute the motor speed values 
        Args:
            imu (sensor_msgs/Imu): sensor message containing linear accelerations and angular velocities
        """
        actual.w_z = imu.angular_velocity.y * math.pi / 180
        actual.ax_current = imu.linear_acceleration.x

        t.t_current = time()
        t.delta_t = t.t_current - t.t_prev

        actual.v_x += (actual.ax_current + actual.ax_prev) * t.delta_t / 2
        
        t.t_prev = t.t_current
        actual.ax_prev = actual.ax_current
        
        vx = ref.v_x * PARAM.kp_x

        wz = pid_wz.compute(ref.w_z, actual.w_z)

        phi_r, phi_l = self.robot.kinematic(vx, wz)
        max_phi, min_phi = self.robot.find_phi_boudary_values(PARAM)

        phi_r, phi_l = self.robot.sturate(phi_r, phi_l, max_phi, min_phi)

        pwm_r = map_from_to(phi_r, min_phi, max_phi, PARAM.min_speed, PARAM.max_speed)
        pwm_l = map_from_to(phi_l, min_phi, max_phi, PARAM.min_speed, PARAM.max_speed)

        pwm_r = round(pwm_r)
        pwm_l = round(pwm_l)

        self.motor_speeds.data = [pwm_r, pwm_l]
        self.motors.publish(self.motor_speeds)
        rospy.loginfo(f"motor speeds: {self.motor_speeds.data}  \nactual_w_z: {actual.w_z} \nw_z: {wz}")

    def stopAll(self) -> None:
        """Stop all motors publish zeros to the motors"""
        self.motor_speeds.data = [0, 0]
        self.motors.publish(self.motor_speeds)


if __name__ == "__main__":
    Node()