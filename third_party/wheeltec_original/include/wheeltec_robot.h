#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <string>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>
#include <chrono>
#include <memory>
#include <functional>

#include <serial/serial.h>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;

// Macro definition
#define SEND_DATA_CHECK   1
#define READ_DATA_CHECK   0
#define FRAME_HEADER      0X7B
#define FRAME_TAIL        0X7D
#define RECEIVE_DATA_SIZE 24
#define SEND_DATA_SIZE    11
#define PI                3.1415926f

#define GYROSCOPE_RATIO   0.00026644f
#define ACCEl_RATIO       1671.84f

extern sensor_msgs::msg::Imu Mpu6050;

// Covariance matrix
const double odom_pose_covariance[36]   = {
  1e-3, 0,    0,   0,   0,    0,
  0,    1e-3, 0,   0,   0,    0,
  0,    0,    1e6, 0,   0,    0,
  0,    0,    0,   1e6, 0,    0,
  0,    0,    0,   0,   1e6,  0,
  0,    0,    0,   0,   0,    1e3
};

const double odom_pose_covariance2[36]  = {
  1e-9, 0,    0,    0,   0,    0,
  0,    1e-3, 1e-9, 0,   0,    0,
  0,    0,    1e6,  0,   0,    0,
  0,    0,    0,    1e6, 0,    0,
  0,    0,    0,    0,   1e6,  0,
  0,    0,    0,    0,   0,    1e-9
};

const double odom_twist_covariance[36]  = {
  1e-3, 0,    0,   0,   0,    0,
  0,    1e-3, 0,   0,   0,    0,
  0,    0,    1e6, 0,   0,    0,
  0,    0,    0,   1e6, 0,    0,
  0,    0,    0,   0,   1e6,  0,
  0,    0,    0,   0,   0,    1e3
};

const double odom_twist_covariance2[36] = {
  1e-9, 0,    0,    0,   0,    0,
  0,    1e-3, 1e-9, 0,   0,    0,
  0,    0,    1e6,  0,   0,    0,
  0,    0,    0,    1e6, 0,    0,
  0,    0,    0,    0,   1e6,  0,
  0,    0,    0,    0,   0,    1e-9
};

typedef struct __Vel_Pos_Data_
{
  float X;
  float Y;
  float Z;
} Vel_Pos_Data;

typedef struct __MPU6050_DATA_
{
  short accele_x_data;
  short accele_y_data;
  short accele_z_data;
  short gyros_x_data;
  short gyros_y_data;
  short gyros_z_data;
} MPU6050_DATA;

typedef struct _SEND_DATA_
{
  uint8_t tx[SEND_DATA_SIZE];
  float X_speed;
  float Y_speed;
  float Z_speed;
  unsigned char Frame_Tail;
} SEND_DATA;

typedef struct _RECEIVE_DATA_
{
  uint8_t rx[RECEIVE_DATA_SIZE];
  uint8_t Flag_Stop;
  unsigned char Frame_Header;
  float X_speed;
  float Y_speed;
  float Z_speed;
  float Power_Voltage;
  unsigned char Frame_Tail;
} RECEIVE_DATA;

class turn_on_robot : public rclcpp::Node
{
public:
  turn_on_robot();
  ~turn_on_robot();

  void Control();
  serial::Serial Stm32_Serial;

private:
  rclcpp::Time _Now, _Last_Time;
  float Sampling_Time;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;
  void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher;

  rclcpp::TimerBase::SharedPtr control_timer;

  void Publish_Odom();
  void Publish_ImuSensor();
  void Publish_Voltage();

  bool Get_Sensor_Data();
  bool Get_Sensor_Data_New();
  void Reconnect_Serial();
  unsigned char Check_Sum(unsigned char Count_Number, unsigned char mode);
  short IMU_Trans(uint8_t Data_High, uint8_t Data_Low);
  float Odom_Trans(uint8_t Data_High, uint8_t Data_Low);

  string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id;
  int serial_baud_rate;
  RECEIVE_DATA Receive_Data;
  SEND_DATA Send_Data;

  Vel_Pos_Data Robot_Pos;
  Vel_Pos_Data Robot_Vel;
  MPU6050_DATA Mpu6050_Data;
  float Power_voltage;
  float odom_x_scale, odom_y_scale, odom_z_scale_positive, odom_z_scale_negative;
};

#endif