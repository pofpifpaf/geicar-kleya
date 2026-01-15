#include <memory>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

using namespace std::chrono_literals;

class MagCalibrator : public rclcpp::Node {
public:
    MagCalibrator() : Node("mag_calibrator_node"), count_(0) {
        // Parameter for scaling (10e7 as discussed)
        this->declare_parameter("mag_scale_factor", 10000000.0);

        subscription_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
            "/imu/mag", 10, std::bind(&MagCalibrator::topic_callback, this, std::placeholders::_1));

        // Timer to countdown 30 seconds
        start_time_ = this->now();
        timer_ = this->create_wall_timer(1s, std::bind(&MagCalibrator::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "CALIBRATION STARTED! You have 30 seconds.");
        RCLCPP_INFO(this->get_logger(), "Rotate the sensor in a figure-8 or 3D sphere now...");
    }

private:
    void topic_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
        double scale = this->get_parameter("mag_scale_factor").as_double();
        
        float x = msg->magnetic_field.x * scale;
        float y = msg->magnetic_field.y * scale;
        float z = msg->magnetic_field.z * scale;

        // Initialize or update Min/Max
        if (first_sample_) {
            min_x_ = max_x_ = x;
            min_y_ = max_y_ = y;
            min_z_ = max_z_ = z;
            first_sample_ = false;
        } else {
            min_x_ = std::min(min_x_, x); max_x_ = std::max(max_x_, x);
            min_y_ = std::min(min_y_, y); max_y_ = std::max(max_y_, y);
            min_z_ = std::min(min_z_, z); max_z_ = std::max(max_z_, z);
        }
        count_++;
    }

    void timer_callback() {
        auto now = this->now();
        auto elapsed = (now - start_time_).seconds();
        int remaining = 30 - static_cast<int>(elapsed);

        if (remaining > 0) {
            if (remaining % 5 == 0 || remaining < 5) {
                RCLCPP_INFO(this->get_logger(), "Time remaining: %d seconds... (Samples: %d)", remaining, count_);
            }
        } else {
            finish_calibration();
        }
    }

    void finish_calibration() {
        if (count_ < 10) {
            RCLCPP_ERROR(this->get_logger(), "Calibration failed: Not enough data received!");
        } else {
            // Hard-Iron Offset = (Max + Min) / 2
            float off_x = (max_x_ + min_x_) / 2.0f;
            float off_y = (max_y_ + min_y_) / 2.0f;
            float off_z = (max_z_ + min_z_) / 2.0f;

            std::cout << "\n================================================" << std::endl;
            std::cout << "CALIBRATION COMPLETE" << std::endl;
            std::cout << "================================================" << std::endl;
            std::cout << "Raw Ranges (mGauss):" << std::endl;
            std::cout << "X: [" << min_x_ << " to " << max_x_ << "]" << std::endl;
            std::cout << "Y: [" << min_y_ << " to " << max_y_ << "]" << std::endl;
            std::cout << "Z: [" << min_z_ << " to " << max_z_ << "]" << std::endl;
            std::cout << "------------------------------------------------" << std::endl;
            std::cout << "USE THESE VALUES IN YOUR ECOMPASS NODE:" << std::endl;
            std::cout << "mag_offset_x: " << off_x << std::endl;
            std::cout << "mag_offset_y: " << off_y << std::endl;
            std::cout << "mag_offset_z: " << off_z << std::endl;
            std::cout << "================================================\n" << std::endl;
        }
        
        rclcpp::shutdown(); // This terminates the node
    }

    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    
    bool first_sample_ = true;
    float min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
    int count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MagCalibrator>());
    return 0;
}