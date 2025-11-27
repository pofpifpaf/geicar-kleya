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

// Serial

#define PORT "/dev/ttyACM0"
#define BAUD_RATE (B115200)
#define TIMER_PERIOD (25)

// SOH

#define GET_VERSION (0x10)
#define VERSION (0x11)
#define RESET_HEADING (0x20)
#define CALIBRATE_MAG (0x21)
#define MOTION (0x30)
#define ECOMPASS (0x31)
#define ENV (0x32)

// Misc

#define FIRST_BYTE (0x01)
#define HEADER_SIZE (5)

#define FRAME_BUFFER_LENGTH (100)

// States

#define INITIAL_STATE_SOH (0)
#define STATE_FRAME_ID (1)
#define STATE_FRAME_LENGTH_FIRST_BYTE (2)
#define STATE_FRAME_LENGTH_SECOND_BYTE (3)
#define STATE_FRAME_DATA (4)
#define STATE_FRAME_COMPLETE (120)

// Offsets

// -- MOTION

#define OFFSET_ACCELEROMETER_X (0)
#define OFFSET_ACCELEROMETER_Y (4)
#define OFFSET_ACCELEROMETER_Z (8)

#define OFFSET_GYROSCOPE_X (12)
#define OFFSET_GYROSCOPE_Y (16)
#define OFFSET_GYROSCOPE_Z (20)

#define OFFSET_MAG_X (24)
#define OFFSET_MAG_Y (28)
#define OFFSET_MAG_Z (32)

// -- ECOMPASS

#define OFFSET_YAW (0)
#define OFFSET_PITCH (4)
#define OFFSET_ROLL (8)
#define OFFSET_HEADING (12)
#define OFFSET_HEADING_VALID (16)

// -- ENV

#define OFFSET_TEMPERATURE (0)
#define OFFSET_PRESSURE (4)
#define OFFSET_HUMIDITY (8)

using std::placeholders::_1;

class imu_rx : public rclcpp::Node {
public:
  imu_rx()
  : Node("imu_rx_node")
  {
    publisher_imu_data_raw_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    fd_ = openSerialPort(PORT);
    if (!configureSerialPort(fd_, BAUD_RATE))
    {
      RCLCPP_FATAL(this->get_logger(), "Error configuring Serial Port");
      error = true;
    }



    reader_thread_ = std::thread(&imu_rx::serialLoop, this);

    if (!error)
    {
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

  bool error = false;



  struct ecompass 
  {
    float yaw;
    float pitch;
    float roll;
    float heading;
    uint8_t heading_valid;
  };

  struct environmental
  {
    float temperature;
    float pressure;
    float humidity;
  };

  struct ecompass imu_ecompass;
  struct environmental imu_env;

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

  bool checkChecksum()
  {
    // Check if the frame is correct
    uint8_t sum = 0;
    for (uint16_t i = 0; i < frame_length - 2; i++) 
    {
      sum = static_cast<uint8_t>(sum + frame[i]);
    }

    RCLCPP_INFO(this->get_logger(), "Checksum found is : 0x%02X", sum);
    RCLCPP_INFO(this->get_logger(), "Actual checksum is : 0x%02X", frame[frame_length - 1]);

    return sum == frame[frame_length - 1];

  }

  float get_float(uint8_t offset)
  {
    float out;

    memcpy(&out, &frame[STATE_FRAME_DATA + offset], sizeof(float));
    return out;
  }

  void diffuseFrame()
  {

    // Diffuse frame info on topic

    

    switch (frame[STATE_FRAME_ID])
    {
    case VERSION :
      RCLCPP_INFO(this->get_logger(), "Received version"); // ADD version
      break;

    case MOTION :
      if (frame_length >= 33)
      {
        auto imu_raw_msg = sensor_msgs::msg::Imu();

        imu_raw_msg.linear_acceleration.x = get_float(OFFSET_ACCELEROMETER_X) * pow(9.80665,-2);    //Conversion to [m/s²]
        imu_raw_msg.linear_acceleration.y = get_float(OFFSET_ACCELEROMETER_Y) * pow(9.80665,-2);    //Conversion to [m/s²]
        imu_raw_msg.linear_acceleration.z = get_float(OFFSET_ACCELEROMETER_Z) * pow(9.80665,-2);    //Conversion to [m/s²]

        imu_raw_msg.angular_velocity.x = get_float(OFFSET_GYROSCOPE_X) * pow(1.7453,-5);            //Conversion to [rad/s]
        imu_raw_msg.angular_velocity.y = get_float(OFFSET_GYROSCOPE_Y) * pow(1.7453,-5);            //Conversion to [rad/s]
        imu_raw_msg.angular_velocity.z = get_float(OFFSET_GYROSCOPE_Z) * pow(1.7453,-5);            //Conversion to [rad/s]

        imu_raw_msg.header.stamp = rclcpp::Clock().now();
        publisher_imu_data_raw_->publish(imu_raw_msg);

        // auto imu_mag_msg = sensor_msgs::msg::MagneticField();

        // imu_mag_msg.magnetic_field.x = get_float(OFFSET_MAG_X) * pow(10,-7);
        // imu_mag_msg.magnetic_field.y = get_float(OFFSET_MAG_Y) * pow(10,-7);
        // imu_mag_msg.magnetic_field.z = get_float(OFFSET_MAG_Z) * pow(10,-7);

        // imu_mag_msg.header.stamp = rclcpp::Clock().now();
        // publisher_imu_mag_->publish(imu_mag_msg); 
      }
      else { RCLCPP_ERROR(this->get_logger(), "invalid frame length"); }
      break;

    case ECOMPASS : 
      if (frame_length >= 18)
      {
        imu_ecompass.yaw = get_float(0);
        imu_ecompass.pitch = get_float(4);
        imu_ecompass.roll = get_float(8);
        imu_ecompass.heading = get_float(12);

        imu_ecompass.heading_valid = frame[STATE_FRAME_DATA + 15];
      }
      else { RCLCPP_ERROR(this->get_logger(), "invalid frame length"); }
      break;

    case ENV :
      if (frame_length >= 17)
      {
        imu_env.temperature = get_float(0);
        imu_env.pressure = get_float(4);
        imu_env.humidity = get_float(8);
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

  size_t index = 0; // ADD AS MEMBER

  void serialLoop() 
  {
    uint8_t byte = 0;
    
    while (running_) 
    {
      if (state == STATE_FRAME_COMPLETE) 
      {

        if (checkChecksum())
        {
          RCLCPP_INFO(this->get_logger(), "Checksum successful - Diffusing frame : ID = 0x%02X ", frame[STATE_FRAME_ID]);
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
        // RCLCPP_INFO(this->get_logger(), "Received 0x%02X", byte);
        switch (state)
        {
        case INITIAL_STATE_SOH :
          index = 0;
          if (byte == FIRST_BYTE)
          {
            RCLCPP_INFO(this->get_logger(), "Serial frame : Valid first byte 0x%02X !!", byte);
            frame[index++] = byte;
            state = STATE_FRAME_ID;
          }
          else
          {
            // Send reset heading
            RCLCPP_INFO(this->get_logger(), "Serial frame : Invalid first byte 0x%02X - Resetting Header", byte);
          }
          break;

        case STATE_FRAME_ID :
          frame[index++] = byte;
          state = STATE_FRAME_LENGTH_FIRST_BYTE;
          RCLCPP_INFO(this->get_logger(), "Received frame ID 0x%02X", byte);
          break;

        case STATE_FRAME_LENGTH_FIRST_BYTE : 
          // Received third byte (frame length - first byte)
          frame_length = byte;
          frame[index++] = byte;
          state = STATE_FRAME_LENGTH_SECOND_BYTE;
          // RCLCPP_INFO(this->get_logger(), "Received frame length first byte 0x%02X", byte);
          break;

        case STATE_FRAME_LENGTH_SECOND_BYTE : 
          // Received fourth byte (frame length - second byte)
          frame_length = (byte | (frame_length << 8)) + HEADER_SIZE;
          frame[index++] = byte;
          state++;
          // RCLCPP_INFO(this->get_logger(), "Received frame length second byte 0x%02X", byte);
          // RCLCPP_INFO(this->get_logger(), "Received frame length %d", frame_length);
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