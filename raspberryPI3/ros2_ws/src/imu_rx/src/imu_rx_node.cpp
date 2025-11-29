#include <cstdio>

#include "std_srvs/srv/empty.hpp"

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <algorithm>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h> 
#include <iostream>
#include <string> 

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include "interfaces/msg/e_compass.hpp"
#include "interfaces/msg/environment_data.hpp"

#include "imu_rx/imu_rx_node.hpp"

using std::placeholders::_1;

class imu_rx : public rclcpp::Node {
public:
  imu_rx()
  : Node("imu_rx_node")
  {
    publisher_imu_data_raw_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    publisher_imu_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);

    publisher_imu_ecompass_ = this->create_publisher<interfaces::msg::ECompass>("imu/ecompass", 10);
    publisher_imu_env_ = this->create_publisher<interfaces::msg::EnvironmentData>("imu/env", 10);

    fd_ = openSerialPort(PORT);
    if (!configureSerialPort(fd_, BAUD_RATE))
    {
      RCLCPP_FATAL(this->get_logger(), "Error configuring Serial Port");
      error = true;
    }

    if (!error)
    {
      reader_thread_ = std::thread(&imu_rx::serialLoop, this);
      RCLCPP_INFO(this->get_logger(), "Successfully connected to Serial port");
      RCLCPP_INFO(this->get_logger(), "imu_rx_node READY");
    }
  }

  ~imu_rx()
  {
      running_ = false;
      if (reader_thread_.joinable())
          reader_thread_.join();
  }

private:

  uint8_t checksum = 0;
  uint8_t frame[FRAME_BUFFER_LENGTH] = { 0 };
  uint16_t frame_length = 0;
  uint16_t state = 0;
  size_t index = 0; 

  bool error = false;

  // Opens the serial port
  int openSerialPort(const char* port)
  {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0)
    {    
      RCLCPP_FATAL(this->get_logger(), "Unable to open Serial Port");
      return -1;
    }

    return fd;
  }

  // Configuring the serial port
  bool configureSerialPort(int fd, int speed)
  {
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0) 
    {
      RCLCPP_FATAL(this->get_logger(), "Error from tcgetattr");
      return false;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0; // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0; // no remapping, no delays
    tty.c_cc[VMIN] = 0; // read doesn't block
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) 
    {
      RCLCPP_FATAL(this->get_logger(), "Error setting terminal attributes");
      return false;
    }

    return true;
  }

  // Checks if checksum is correct
  bool checkChecksum()
  {
    // Check if the frame is correct
    uint8_t sum = 0;
    for (uint16_t i = 0; i < frame_length - 2; i++) 
    {
      sum = static_cast<uint8_t>(sum + frame[i]);
    }

    return sum == frame[frame_length - 1];

  }

  // Function to extract 4 data bytes into a float
  float get_float(uint8_t offset)
  {
    float out;
    memcpy(&out, &frame[STATE_FRAME_DATA + offset], sizeof(float));
    return out;
  }

  // Function to send frames into their appropriate topic
  void diffuseFrame()
  {

    // Diffuse frame info on topic
    switch (frame[STATE_FRAME_ID])
    {
    case VERSION :
      if (frame_length == FRAME_VERSION_SIZE)
      {
        RCLCPP_INFO(this->get_logger(), "Received version %u.%u.%u", frame[STATE_FRAME_DATA + 0], 
                                                                    frame[STATE_FRAME_DATA + 1], 
                                                                    frame[STATE_FRAME_DATA + 2]);
      }
      break;

    case MOTION :
      if (frame_length == FRAME_MOTION_SIZE)
      {
        auto imu_raw_msg = sensor_msgs::msg::Imu();

        imu_raw_msg.linear_acceleration.x = get_float(OFFSET_ACCELEROMETER_X) * G_CONSTANT;    //Conversion to [m/s²]
        imu_raw_msg.linear_acceleration.y = get_float(OFFSET_ACCELEROMETER_Y) * G_CONSTANT;    //Conversion to [m/s²]
        imu_raw_msg.linear_acceleration.z = get_float(OFFSET_ACCELEROMETER_Z) * G_CONSTANT;    //Conversion to [m/s²]

        imu_raw_msg.angular_velocity.x = get_float(OFFSET_GYROSCOPE_X) * RAD_S_CONSTANT;       //Conversion to [rad/s]
        imu_raw_msg.angular_velocity.y = get_float(OFFSET_GYROSCOPE_Y) * RAD_S_CONSTANT;       //Conversion to [rad/s]
        imu_raw_msg.angular_velocity.z = get_float(OFFSET_GYROSCOPE_Z) * RAD_S_CONSTANT;       //Conversion to [rad/s]

        imu_raw_msg.header.stamp = rclcpp::Clock().now();
        publisher_imu_data_raw_->publish(imu_raw_msg);

        auto imu_mag_msg = sensor_msgs::msg::MagneticField();

        imu_mag_msg.magnetic_field.x = get_float(OFFSET_MAG_X) * pow(10,-7);
        imu_mag_msg.magnetic_field.y = get_float(OFFSET_MAG_Y) * pow(10,-7);
        imu_mag_msg.magnetic_field.z = get_float(OFFSET_MAG_Z) * pow(10,-7);

        imu_mag_msg.header.stamp = rclcpp::Clock().now();
        publisher_imu_mag_->publish(imu_mag_msg); 
      }
      else { RCLCPP_ERROR(this->get_logger(), "invalid frame length"); }
      break;

    case ECOMPASS : 
      if (frame_length == FRAME_ECOMPASS_SIZE)
      {
        auto ecompass_msg = interfaces::msg::ECompass();

        ecompass_msg.yaw = get_float(OFFSET_YAW);
        ecompass_msg.pitch = get_float(OFFSET_PITCH);
        ecompass_msg.roll = get_float(OFFSET_ROLL);
        ecompass_msg.heading = get_float(OFFSET_HEADING);

        ecompass_msg.heading_valid = frame[STATE_FRAME_DATA + OFFSET_HEADING_VALID];

        publisher_imu_ecompass_->publish(ecompass_msg); 
      }
      else { RCLCPP_ERROR(this->get_logger(), "invalid frame length"); }
      break;

    case ENV :
      if (frame_length == FRAME_ENV_SIZE)
      {
        auto env_msg = interfaces::msg::EnvironmentData();

        env_msg.temperature = get_float(OFFSET_TEMPERATURE);
        env_msg.pressure = get_float(OFFSET_PRESSURE);
        env_msg.humidity = get_float(OFFSET_HUMIDITY);

        publisher_imu_env_->publish(env_msg); 
      }
      else { RCLCPP_ERROR(this->get_logger(), "invalid frame length"); }
      break;

    default :
      RCLCPP_ERROR(this->get_logger(), "invalid frame ID");
      break;
    }

    state = INITIAL_STATE_SOH;
    frame_length = 0;
  }

  // Loop that receives each byte and transforms it
  void serialLoop() 
  {
    uint8_t byte = 0;
    
    while (running_) 
    {
      if (state == STATE_FRAME_COMPLETE) 
      {

        if (checkChecksum())
        {
          RCLCPP_DEBUG(this->get_logger(), "Checksum successful - Diffusing frame : ID = 0x%02X ", frame[STATE_FRAME_ID]);
          diffuseFrame(); 
        }
        else 
        {
          RCLCPP_ERROR(this->get_logger(), "Error : frame failed checksum");
          state = INITIAL_STATE_SOH;
          memset(frame, 0, sizeof(frame));
          index = 0;
          frame_length = 0;
        }
      }

      int n = read(fd_, &byte, sizeof(byte));

      if (n > 0) 
      {
        switch (state)
        {
        case INITIAL_STATE_SOH :
          index = 0;
          if (byte == FIRST_BYTE)
          {
            frame[index++] = byte;
            state = STATE_FRAME_ID;
          }
          else
          {
            // Send reset heading
            RCLCPP_ERROR(this->get_logger(), "Serial frame : Invalid first byte 0x%02X - Resetting Header", byte);
          }
          break;

        case STATE_FRAME_ID :
          frame[index++] = byte;
          state = STATE_FRAME_LENGTH_FIRST_BYTE;
          break;

        case STATE_FRAME_LENGTH_FIRST_BYTE : 
          // Received third byte (frame length - first byte)
          frame_length = byte;
          frame[index++] = byte;
          state = STATE_FRAME_LENGTH_SECOND_BYTE;
          break;

        case STATE_FRAME_LENGTH_SECOND_BYTE : 
          // Received fourth byte (frame length - second byte)
          frame_length = (byte | (frame_length << 8)) + HEADER_SIZE;
          frame[index++] = byte;
          state++;
          break;

        default : 
          if (state > 0 && index <= frame_length && index < FRAME_BUFFER_LENGTH)
          {
            
            frame[index++] = byte;
          }

          if (index == frame_length) { state = STATE_FRAME_COMPLETE; }
          else { state++; }

          break;
          
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_data_raw_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_imu_mag_;
  rclcpp::Publisher<interfaces::msg::ECompass>::SharedPtr publisher_imu_ecompass_;
  rclcpp::Publisher<interfaces::msg::EnvironmentData>::SharedPtr publisher_imu_env_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::thread reader_thread_;
  std::atomic<bool> running_{true};

  int fd_; 
   
};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<imu_rx>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}