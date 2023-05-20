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
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;

ros::Publisher pub_imu("/imu", &imu_msg);


const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;


// motor driver library
AF_DCMotor Right_Motor(3);
AF_DCMotor Left_Motor(1);

// velocities and directions
int PWM_r = 0;
int PWM_l = 0;
int Dir_r = RELEASE;
int Dir_l = RELEASE;

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
}

ros::Subscriber<std_msgs::Int16MultiArray> motors("/motor_speeds", &motorsCb);

void setup() 
{
  attachInterrupt(digitalPinToInterrupt(19),pub_Rturns,RISING);
  attachInterrupt(digitalPinToInterrupt(18),pub_Lturns,FALLING);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  nh.initNode();
  nh.advertise(pub_imu);
  nh.subscribe(motors);
}

void loop() 
{
  imu();
  nh.spinOnce();
  delay(50);
}

void pub_Rturns(){
//  if (Dir_r == FORWARD) Rturns.data += 1;
//  else if (Dir_r == BACKWARD) Rturns.data -= 1;
  }

void pub_Lturns(){
//  if (Dir_l == FORWARD) Lturns.data += 1;
//  else if (Dir_l == BACKWARD) Lturns.data -= 1;
  }

void imu(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  
  AccX = AccX;// - error[0];
  AccY = AccY;// - error[1];
  AccZ = AccZ;
  
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI); //- 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI); //+ 1.58; // AccErrorY ~(-1.58)
  
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX;// - error[2]; // GyroErrorX ~(-0.56)
  GyroY = GyroY;// - error[3]; // GyroErrorY ~(2)
  GyroZ = GyroZ;// - error[4]; // GyroErrorZ ~ (-0.8)

  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;

  roll = gyroAngleX;
  pitch = gyroAngleY;

  imu_msg.orientation.x = roll;
  imu_msg.orientation.y = pitch;
  imu_msg.orientation.z = yaw;
  imu_msg.orientation.w = 0.00;
  imu_msg.angular_velocity.x = GyroX;
  imu_msg.angular_velocity.y = GyroY;
  imu_msg.angular_velocity.z = GyroZ;
  imu_msg.linear_acceleration.x = AccX;
  imu_msg.linear_acceleration.y = AccY;
  imu_msg.linear_acceleration.z = AccZ;
  imu_msg.header.stamp = nh.now();

  pub_imu.publish(&imu_msg);
}

void odom(){
  
}
