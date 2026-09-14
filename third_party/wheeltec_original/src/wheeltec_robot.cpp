#include "wheeltec_robot.h"
#include "Quaternion_Solution.h"

sensor_msgs::msg::Imu Mpu6050;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto robot_control = std::make_shared<turn_on_robot>();
  try {
    rclcpp::spin(robot_control);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(robot_control->get_logger(), "unhandled exception, shutting down: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}

short turn_on_robot::IMU_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  short transition_16 = 0;
  transition_16 |= Data_High << 8;
  transition_16 |= Data_Low;
  return transition_16;
}

float turn_on_robot::Odom_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  float data_return;
  short transition_16 = 0;
  transition_16 |= Data_High << 8;
  transition_16 |= Data_Low;
  data_return = (transition_16 / 1000) + (transition_16 % 1000) * 0.001;
  return data_return;
}

void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
  short transition = 0;
  const bool is_nonzero_cmd =
    std::fabs(twist_aux->linear.x) > 1e-4 ||
    std::fabs(twist_aux->linear.y) > 1e-4 ||
    std::fabs(twist_aux->angular.z) > 1e-4;

  Send_Data.tx[0] = FRAME_HEADER;
  Send_Data.tx[1] = 0;
  Send_Data.tx[2] = 0;

  transition = static_cast<short>(twist_aux->linear.x * 1000);
  Send_Data.tx[4] = transition;
  Send_Data.tx[3] = transition >> 8;

  transition = static_cast<short>(twist_aux->linear.y * 1000);
  Send_Data.tx[6] = transition;
  Send_Data.tx[5] = transition >> 8;

  transition = static_cast<short>(twist_aux->angular.z * 1000);
  Send_Data.tx[8] = transition;
  Send_Data.tx[7] = transition >> 8;

  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK);
  Send_Data.tx[10] = FRAME_TAIL;

  try
  {
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
    if (is_nonzero_cmd) {
      RCLCPP_INFO(
        this->get_logger(),
        "cmd_vel serial write: vx=%.3f vy=%.3f wz=%.3f bytes=[%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X]",
        twist_aux->linear.x,
        twist_aux->linear.y,
        twist_aux->angular.z,
        Send_Data.tx[0],
        Send_Data.tx[1],
        Send_Data.tx[2],
        Send_Data.tx[3],
        Send_Data.tx[4],
        Send_Data.tx[5],
        Send_Data.tx[6],
        Send_Data.tx[7],
        Send_Data.tx[8],
        Send_Data.tx[9],
        Send_Data.tx[10]);
    }
  }
  catch (serial::IOException & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port");
  }
}

void turn_on_robot::Publish_ImuSensor()
{
  sensor_msgs::msg::Imu Imu_Data_Pub;
  Imu_Data_Pub.header.stamp = this->now();
  Imu_Data_Pub.header.frame_id = gyro_frame_id;

  Imu_Data_Pub.orientation.x = Mpu6050.orientation.x;
  Imu_Data_Pub.orientation.y = Mpu6050.orientation.y;
  Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
  Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;

  Imu_Data_Pub.orientation_covariance[0] = 1e6;
  Imu_Data_Pub.orientation_covariance[4] = 1e6;
  Imu_Data_Pub.orientation_covariance[8] = 1e-6;

  Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x;
  Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
  Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;

  Imu_Data_Pub.angular_velocity_covariance[0] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;

  Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x;
  Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
  Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;

  imu_publisher->publish(Imu_Data_Pub);
}

void turn_on_robot::Publish_Odom()
{
  tf2::Quaternion q;
  q.setRPY(0, 0, Robot_Pos.Z);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = this->now();
  odom.header.frame_id = odom_frame_id;
  odom.pose.pose.position.x = Robot_Pos.X;
  odom.pose.pose.position.y = Robot_Pos.Y;
  odom.pose.pose.position.z = Robot_Pos.Z;
  odom.pose.pose.orientation = odom_quat;

  odom.child_frame_id = robot_frame_id;
  odom.twist.twist.linear.x = Robot_Vel.X;
  odom.twist.twist.linear.y = Robot_Vel.Y;
  odom.twist.twist.angular.z = Robot_Vel.Z;

  if (Robot_Vel.X == 0 && Robot_Vel.Y == 0 && Robot_Vel.Z == 0)
  {
    memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2));
    memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
  }
  else
  {
    memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance));
    memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
  }

  odom_publisher->publish(odom);
}

void turn_on_robot::Publish_Voltage()
{
  std_msgs::msg::Float32 voltage_msgs;
  static float Count_Voltage_Pub = 0;

  if (Count_Voltage_Pub++ > 10)
  {
    Count_Voltage_Pub = 0;
    voltage_msgs.data = Power_voltage;
    voltage_publisher->publish(voltage_msgs);
  }
}

unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number, unsigned char mode)
{
  unsigned char check_sum = 0, k;

  if (mode == 0)
  {
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Receive_Data.rx[k];
    }
  }
  if (mode == 1)
  {
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Send_Data.tx[k];
    }
  }
  return check_sum;
}

bool turn_on_robot::Get_Sensor_Data()
{
  short transition_16 = 0, j = 0, Header_Pos = 0, Tail_Pos = 0;
  static int flag_error = 0, temp = 1;
  uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE] = {0}, Receive_Data_Tr[temp] = {0};

  if (flag_error == 0)
    Stm32_Serial.read(Receive_Data_Pr, sizeof(Receive_Data_Pr));
  else if (flag_error == 1)
  {
    Stm32_Serial.read(Receive_Data_Tr, sizeof(Receive_Data_Tr));
    flag_error = 0;
  }

  for (j = 0; j < 24; j++)
  {
    if (Receive_Data_Pr[j] == FRAME_HEADER)
      Header_Pos = j;
    else if (Receive_Data_Pr[j] == FRAME_TAIL)
      Tail_Pos = j;
  }

  if (Tail_Pos == (Header_Pos + 23))
  {
    memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
    flag_error = 0;
  }
  else if (Header_Pos == (1 + Tail_Pos))
  {
    temp = Header_Pos;
    flag_error = 1;
    return false;
  }
  else
  {
    return false;
  }

  Receive_Data.Frame_Header = Receive_Data.rx[0];
  Receive_Data.Frame_Tail = Receive_Data.rx[23];

  if (Receive_Data.Frame_Header == FRAME_HEADER)
  {
    if (Receive_Data.Frame_Tail == FRAME_TAIL)
    {
      if (Receive_Data.rx[22] == Check_Sum(22, READ_DATA_CHECK))
      {
        Receive_Data.Flag_Stop = Receive_Data.rx[1];
        Robot_Vel.X = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]);
        Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]);
        Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]);

        Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]);
        Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10], Receive_Data.rx[11]);
        Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]);
        Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14], Receive_Data.rx[15]);
        Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16], Receive_Data.rx[17]);
        Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]);

        Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;

        Mpu6050.angular_velocity.x = Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.y = Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.z = Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

        transition_16 = 0;
        transition_16 |= Receive_Data.rx[20] << 8;
        transition_16 |= Receive_Data.rx[21];
        Power_voltage = transition_16 / 1000 + (transition_16 % 1000) * 0.001;

        return true;
      }
    }
  }
  return false;
}

bool turn_on_robot::Get_Sensor_Data_New()
{
  // Drain every byte currently available in the serial input buffer in this
  // call instead of reading exactly one byte per 10ms Control() tick. The
  // previous one-byte-per-tick loop required 24 ticks (~240ms) to assemble
  // a 24-byte frame, capping /odom at ~4Hz even though STM32 sends at 20Hz.
  short transition_16 = 0;
  uint8_t check = 0;
  static int count = 0;

  size_t available = 0;
  try {
    available = Stm32_Serial.available();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "serial available() failed (%s), closing port", e.what());
    count = 0;
    try { if (Stm32_Serial.isOpen()) Stm32_Serial.close(); } catch (const std::exception &) {}
    return false;
  }

  if (available == 0) {
    return false;
  }

  bool frame_complete = false;

  while (available-- > 0) {
    uint8_t byte = 0;
    size_t got = 0;
    try {
      got = Stm32_Serial.read(&byte, 1);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "serial read() failed (%s), closing port", e.what());
      count = 0;
      try { if (Stm32_Serial.isOpen()) Stm32_Serial.close(); } catch (const std::exception &) {}
      return false;
    }
    if (got != 1) {
      break;
    }
    Receive_Data.rx[count] = byte;

    if (byte == FRAME_HEADER || count > 0) {
      count++;
    } else {
      count = 0;
    }

    if (count != 24) {
      continue;
    }
    count = 0;

    Receive_Data.Frame_Header = Receive_Data.rx[0];
    Receive_Data.Frame_Tail = Receive_Data.rx[23];

    if (Receive_Data.Frame_Tail != FRAME_TAIL) {
      continue;
    }

    check = Check_Sum(22, READ_DATA_CHECK);
    if (check != Receive_Data.rx[22]) {
      continue;
    }

    Receive_Data.Flag_Stop = Receive_Data.rx[1];
    Robot_Vel.X = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]);
    Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]);
    Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]);

    Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]);
    Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10], Receive_Data.rx[11]);
    Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]);
    Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14], Receive_Data.rx[15]);
    Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16], Receive_Data.rx[17]);
    Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]);

    Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
    Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
    Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;

    Mpu6050.angular_velocity.x = Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
    Mpu6050.angular_velocity.y = Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
    Mpu6050.angular_velocity.z = Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

    transition_16 = 0;
    transition_16 |= Receive_Data.rx[20] << 8;
    transition_16 |= Receive_Data.rx[21];
    Power_voltage = transition_16 / 1000 + (transition_16 % 1000) * 0.001;

    frame_complete = true;
  }

  return frame_complete;
}

void turn_on_robot::Control()
{
  if (!Stm32_Serial.isOpen())
  {
    // Control() runs at 100Hz; throttle reopen attempts to ~1Hz to avoid
    // hammering open() while the USB device is gone.
    static int reconnect_div = 0;
    if (reconnect_div++ % 100 == 0)
    {
      Reconnect_Serial();
    }
    return;
  }

  if (true == Get_Sensor_Data_New())
  {
    _Now = this->now();
    Sampling_Time = (_Now - _Last_Time).seconds();

    Robot_Vel.X = Robot_Vel.X * odom_x_scale;
    Robot_Vel.Y = Robot_Vel.Y * odom_y_scale;
    if (Robot_Vel.Z >= 0)
      Robot_Vel.Z = Robot_Vel.Z * odom_z_scale_positive;
    else
      Robot_Vel.Z = Robot_Vel.Z * odom_z_scale_negative;

    Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
    Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
    Robot_Pos.Z += Robot_Vel.Z * Sampling_Time;

    Quaternion_Solution(
      Mpu6050.angular_velocity.x,
      Mpu6050.angular_velocity.y,
      Mpu6050.angular_velocity.z,
      Mpu6050.linear_acceleration.x,
      Mpu6050.linear_acceleration.y,
      Mpu6050.linear_acceleration.z
    );

    Publish_Odom();
    Publish_ImuSensor();
    Publish_Voltage();

    _Last_Time = _Now;
  }
}

void turn_on_robot::Reconnect_Serial()
{
  try {
    if (Stm32_Serial.isOpen()) {
      Stm32_Serial.close();
    }
  } catch (const std::exception &) {
    // ignore errors while closing a dead port
  }

  try {
    Stm32_Serial.setPort(usart_port_name);
    Stm32_Serial.setBaudrate(serial_baud_rate);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(2000);
    Stm32_Serial.setTimeout(timeout);
    Stm32_Serial.open();
  } catch (const std::exception &) {
    // device not back yet; Control() will retry on the next tick
    RCLCPP_WARN(this->get_logger(), "serial port unavailable, retrying...");
    return;
  }

  if (Stm32_Serial.isOpen()) {
    RCLCPP_INFO(this->get_logger(), "wheeltec_robot serial port reconnected");
  }
}

turn_on_robot::turn_on_robot()
: Node("wheeltec_robot"),
  Sampling_Time(0.0),
  Power_voltage(0.0)
{
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data));
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  this->declare_parameter<std::string>("usart_port_name", "/dev/wheeltec_controller");
  this->declare_parameter<int>("serial_baud_rate", 115200);
  this->declare_parameter<std::string>("odom_frame_id", "odom_combined");
  this->declare_parameter<std::string>("robot_frame_id", "base_footprint");
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");
  this->declare_parameter<double>("odom_x_scale", 1.0);
  this->declare_parameter<double>("odom_y_scale", 1.0);
  this->declare_parameter<double>("odom_z_scale_positive", 1.0);
  this->declare_parameter<double>("odom_z_scale_negative", 1.0);

  usart_port_name = this->get_parameter("usart_port_name").as_string();
  serial_baud_rate = this->get_parameter("serial_baud_rate").as_int();
  odom_frame_id = this->get_parameter("odom_frame_id").as_string();
  robot_frame_id = this->get_parameter("robot_frame_id").as_string();
  gyro_frame_id = this->get_parameter("gyro_frame_id").as_string();
  odom_x_scale = static_cast<float>(this->get_parameter("odom_x_scale").as_double());
  odom_y_scale = static_cast<float>(this->get_parameter("odom_y_scale").as_double());
  odom_z_scale_positive = static_cast<float>(this->get_parameter("odom_z_scale_positive").as_double());
  odom_z_scale_negative = static_cast<float>(this->get_parameter("odom_z_scale_negative").as_double());

  voltage_publisher = this->create_publisher<std_msgs::msg::Float32>("PowerVoltage", 10);
  odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
  imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 20);

  Cmd_Vel_Sub = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    100,
    std::bind(&turn_on_robot::Cmd_Vel_Callback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Data ready");

  try
  {
    Stm32_Serial.setPort(usart_port_name);
    Stm32_Serial.setBaudrate(serial_baud_rate);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(2000);
    Stm32_Serial.setTimeout(timeout);
    Stm32_Serial.open();
  }
  catch (serial::IOException & e)
  {
    RCLCPP_ERROR(this->get_logger(), "wheeltec_robot can not open serial port, Please check the serial port cable!");
  }

  if (Stm32_Serial.isOpen())
  {
    RCLCPP_INFO(this->get_logger(), "wheeltec_robot serial port opened");
  }

  _Last_Time = this->now();

  control_timer = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&turn_on_robot::Control, this)
  );
}

turn_on_robot::~turn_on_robot()
{
  Send_Data.tx[0] = FRAME_HEADER;
  Send_Data.tx[1] = 0;
  Send_Data.tx[2] = 0;

  Send_Data.tx[4] = 0;
  Send_Data.tx[3] = 0;

  Send_Data.tx[6] = 0;
  Send_Data.tx[5] = 0;

  Send_Data.tx[8] = 0;
  Send_Data.tx[7] = 0;
  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK);
  Send_Data.tx[10] = FRAME_TAIL;

  try
  {
    if (Stm32_Serial.isOpen())
    {
      Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
    }
  }
  catch (serial::IOException & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port");
  }

  if (Stm32_Serial.isOpen())
  {
    Stm32_Serial.close();
  }

  RCLCPP_INFO(this->get_logger(), "Shutting down");
}
