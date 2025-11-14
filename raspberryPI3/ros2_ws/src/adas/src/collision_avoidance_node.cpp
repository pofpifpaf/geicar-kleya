#include <cstdio>

#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/ultrasonic.hpp"
#include "std_srvs/srv/empty.hpp"

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

using std::placeholders::_1;

class collision_avoidance : public rclcpp::Node {
public:
  collision_avoidance()
  : Node("collision_avoidance_node")
  {
    subscription_us_data_ = this->create_subscription<interfaces::msg::Ultrasonic>(
    "us_data", 10, std::bind(&collision_avoidance::detectCollisionCallback, this, _1));

    subscription_motors_order_raw_ = this->create_subscription<interfaces::msg::MotorsOrder>(
    "motors_order_raw", 10, std::bind(&collision_avoidance::motorsOrderCallback, this, _1));

    publisher_motors_order_ = this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);

    RCLCPP_INFO(this->get_logger(), "collision_avoidance_node READY");
  }

private:

    struct ultra_data
    {
      int front_left;
      int front_right;
      int front_center;
      int rear_left;
      int rear_right;
      int rear_center;
    };

    struct motors_order
    {
      int right_rear_pwm;
      int left_rear_pwm;
    };

    struct motors_order order;
    struct ultra_data ultras;

    int detectCollisionCallback(const interfaces::msg::Ultrasonic & usData) {

      ultras.front_left = usData.front_left;
      ultras.front_right = usData.front_right;
      ultras.front_center = usData.front_center;

      return detectCollision(ultras);
    }

    int motorsOrderCallback(const interfaces::msg::MotorsOrder & motorsOrder) {

      order.right_rear_pwm = motorsOrder.right_rear_pwm;
      order.left_rear_pwm = motorsOrder.left_rear_pwm;

      return 0;
    }

    int detectCollision(struct ultra_data ultras){
      if (order.right_rear_pwm > 50 || order.left_rear_pwm > 50){
        if (ultras.front_left < 20 || ultras.front_right < 20 || ultras.front_center < 20){
          order.left_rear_pwm = 50;
          order.right_rear_pwm = 50;
          RCLCPP_INFO(this->get_logger(), "Detecting obtacle <20 cm away : Stopping car");
        }
      }

      return 0;
    }

    //Subscribers
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_us_data_;
    rclcpp::Subscription<interfaces::msg::MotorsOrder>::SharedPtr subscription_motors_order_raw_;

    rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr publisher_motors_order_;
   
};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<collision_avoidance>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
