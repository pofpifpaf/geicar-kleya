import rclpy
from rclpy.node import Node
import math

from interfaces.msg import sensor_msgs/Imu # rempalcer par la boussole

class Esp(Node):
    def __init__(self):
        super().__init__('Esp_node')

         self.publisher_Imu = self.create_publisher(sensor_msgs/Imu, 'imu/data', 10) # remplacer par la boussole

        self.subscription = self.create_subscription(sensor_msgs/Imu,'imu/data_raw', self.Imu_callback, 10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info("Esp_node READY")

# remplacer par la boussole
def Imu_callback(self, Imu : MotorsOrder):

        self.motor_right_rear_pwm = motors_order.right_rear_pwm
        self.motor_left_rear_pwm = motors_order.left_rear_pwm


def stability_control(self):
    