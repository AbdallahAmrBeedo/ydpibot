/*
 * This Arduino code implements a ROS node that subscribes to the "/motor_speeds" topic with Int16MultiArray message,
 * and publishes to the "/imu" and "/odom" topics with IMU and encoder odom messages.
 *
 * The node reads the motor speeds from the "/motor_speeds" topic and computes the linear acceleration and angular velocity
 * using the encoder interrupt readings. It publishes the computed linear acceleration, angular velocity, and orientation
 * to the "/imu" topic as an IMU message, and the encoder odom readings to the "/odom" topic as an odom message.
 *
 * This node assumes that the robot has two independently controlled wheels and encoders, and that the motor speeds
 * are given as an array of two integers representing the PWM values.
 *
 * Usage:
 *  - Upload this code to your Arduino board.
 *  - Run the ROS core.
 *  - Start the node with "rosrun rosserial_arduino serial_node.py /dev/ttyACM0".
 *
 * Subscribed Topics:
 *  - /motor_speeds (std_msgs/Int16MultiArray): The motor speeds of the robot.
 *
 * Published Topics:
 *  - /imu (sensor_msgs/Imu): The linear acceleration, angular velocity, and orientation of the robot.
 *  - /odom (nav_msgs/Odometry): The encoder odom readings of the robot.
 *
 * Parameters:
 *  - ~encoder_resolution (int, default: 4096): The encoder resolution in counts per revolution.
 *
 * This code was developed by: 
 *  - Abdallah Amr (Egypt Japan University of Science and Technology)
 *  - Mostafa Osama (Egypt Japan University of Science and Technology)
 *  - Tarek Shohdy (Egypt Japan University of Science and Technology)
 *  - Yomna Omar (Egypt Japan University of Science and Technology)
 */

#include <AFMotor.h>
#include <ros.h>
#include <Wire.h>
#include <ros/time.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int64.h>

ros::NodeHandle nh;

// motor driver library
AF_DCMotor Right_Motor(2);
AF_DCMotor Left_Motor(1);

// velocities and directions
int PWM_r = 0;
int PWM_l = 0;
int Dir_r = RELEASE;
int Dir_l = RELEASE;

float t = 0;

void motorsCb(const std_msgs::Int16MultiArray& PWM){
  PWM_r = PWM.data[0];
  PWM_l = PWM.data[1];
  if (PWM_r == 0){
    Dir_r = RELEASE;
    }
  else{
    Dir_r = PWM_r / abs(PWM_r); // Forward
    if (Dir_r == -1) Dir_r = 2; //Backward
  }
  if (PWM_l == 0){
    Dir_l = RELEASE;
    }
  else{
    Dir_l = PWM_l / abs(PWM_l); // Forward
    if (Dir_l == -1) Dir_l = 2; //Backward    
  }
  
  Right_Motor.setSpeed(abs(PWM_r));
  Left_Motor.setSpeed(abs(PWM_l));
  Right_Motor.run(Dir_r);
  Left_Motor.run(Dir_l);
  delay(10);
}

ros::Subscriber<std_msgs::Int16MultiArray> motors("/motor_speeds", &motorsCb);

std_msgs::Int64 right_ticks;
ros::Publisher right_wheel("/right_ticks", &right_ticks);
std_msgs::Int64 left_ticks;
ros::Publisher left_wheel("/left_ticks", &left_ticks);

void setup() 
{
  attachInterrupt(digitalPinToInterrupt(19),pub_Rturns,RISING);
  attachInterrupt(digitalPinToInterrupt(18),pub_Lturns,FALLING);
  nh.initNode();
  nh.advertise(right_wheel);
  nh.advertise(left_wheel);
  nh.subscribe(motors);
}

void loop() 
{
  nh.spinOnce();
  delay(1);
}

void pub_Rturns(){
  if (Dir_r == FORWARD) right_ticks.data += 1;
  else if (Dir_r == BACKWARD) right_ticks.data -= 1;
  right_wheel.publish(&right_ticks);
  }

void pub_Lturns(){
  if (Dir_l == FORWARD) left_ticks.data += 1;
  else if (Dir_l == BACKWARD) left_ticks.data -= 1;
  left_wheel.publish(&left_ticks);
  }

void odom(){
  
}

float moving_average_filter(float new_reading,float old_reading, float alpha){
 float filtered_reading = alpha*new_reading + (1-alpha)*old_reading;
 return filtered_reading;
  
  }
