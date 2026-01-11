#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "interfaces/msg/e_compass.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using std::placeholders::_1;
using std::placeholders::_2;

class ECompassNode : public rclcpp::Node {
public:
    ECompassNode() : Node("ecompass_node") {
        // Parameters for calibration
        this->declare_parameter("mag_offset_x", 0.0);
        this->declare_parameter("mag_offset_y", 0.0);
        this->declare_parameter("mag_offset_z", 0.0);
        this->declare_parameter("mag_scale_factor", 10000000.0);

        publisher_ = this->create_publisher<interfaces::msg::ECompass>("ecompass", 10);

        // Subscriptions
        imu_sub_.subscribe(this, "/imu/data_raw");
        mag_sub_.subscribe(this, "/imu/mag");

        // ApproximateTime Synchronizer
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), imu_sub_, mag_sub_);
        sync_->registerCallback(std::bind(&ECompassNode::callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "ECompass Node initialized with 10e7 scaling correction.");
    }

private:
    void callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
                  const sensor_msgs::msg::MagneticField::ConstSharedPtr mag_msg) 
    {
        auto out_msg = interfaces::msg::ECompass();

        // 1. Accelerometer -> Tilt (Roll/Pitch)
        float ax = imu_msg->linear_acceleration.x;
        float ay = imu_msg->linear_acceleration.y;
        float az = imu_msg->linear_acceleration.z;

        // Roll: rotation around X axis
        float roll = std::atan2(ay, az);
        // Pitch: rotation around Y axis
        float pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));

        // 2. Magnetometer -> Apply Scale and Offsets
        double scale = this->get_parameter("mag_scale_factor").as_double();
        float mx = (mag_msg->magnetic_field.x * scale) - this->get_parameter("mag_offset_x").as_double();
        float my = (mag_msg->magnetic_field.y * scale) - this->get_parameter("mag_offset_y").as_double();
        float mz = (mag_msg->magnetic_field.z * scale) - this->get_parameter("mag_offset_z").as_double();

        // 3. Tilt Compensation
        // Projects the 3D magnetometer vector onto a flat horizontal plane
        float cosRoll = std::cos(roll);
        float sinRoll = std::sin(roll);
        float cosPitch = std::cos(pitch);
        float sinPitch = std::sin(pitch);

        // These equations level the magnetometer readings
        float Xh = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
        float Yh = my * cosRoll - mz * sinRoll;

        // 4. Heading Calculation
        // atan2(Yh, Xh) gives angle from East (X) in ROS ENU
        float yaw_rad = std::atan2(Yh, Xh);
        float heading_deg = 90.0f - (yaw_rad * 180.0f / M_PI);

        // Wrap to 0-360
        if (heading_deg < 0) heading_deg += 360.0f;
        if (heading_deg >= 360) heading_deg -= 360.0f;

        // 5. Publish
        out_msg.roll = roll * 180.0f / M_PI;
        out_msg.pitch = pitch * 180.0f / M_PI;
        out_msg.yaw = yaw_rad * 180.0f / M_PI;
        out_msg.heading = heading_deg;
        out_msg.heading_valid = 1;

        publisher_->publish(out_msg);
    }

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::MagneticField> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<sensor_msgs::msg::MagneticField> mag_sub_;
    rclcpp::Publisher<interfaces::msg::ECompass>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ECompassNode>());
    rclcpp::shutdown();
    return 0;
}