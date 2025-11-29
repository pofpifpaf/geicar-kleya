import rclpy
from rclpy.node import Node
import math

from interfaces.msg import MotorsOrder #, sensor_msgs/boussole

class Esp(Node):
    def __init__(self):
        super().__init__('Esp_node')

        self.publisher_motors_order = self.create_publisher(MotorsOrder, 'motors_order', 10)
        self.subscription = self.create_subscription(MotorsOrder,'motors_order_raw', self.motors_order_callback, 10)
        #self.subscription = self.create_subscription(sensor_msgs/boussole,'boussole/data_raw', self.boussole_callback, 10)
        self.subscription  # prevent unused variable warning

        self.motor_right_rear_pwm = 50
        self.motor_left_rear_pwm = 50
        self.motor_steering_angle = 0

        #en attendant la boussole
        #self.boussole = angle actuel
        #self.boussole = angle de reference

        self.get_logger().info("Esp_node READY")

    def motors_order_callback(self, motors_order : MotorsOrder):

        self.motor_right_rear_pwm = motors_order.right_rear_pwm
        self.motor_left_rear_pwm = motors_order.left_rear_pwm
        self.motor_steering_angle = motors_order.steering_angle

# remplacer par la boussole
#def boussole_callback(self, boussole : MotorsOrder):
#    self.trajectory_control(self)



def trajectory_control(self):
    """if pas de probleme d'angle :
            msg = MotorsOrder()
            msg.right_rear_pwm = self.motor_right_rear_pwm
            msg.left_rear_pwm = self.motor_left_rear_pwm
            msg.steering_angle = self.motor_steering_angle
            self.publisher_motors_order.publish(msg)
            return
        else 
            angle actuel - angle de reference
            faire +/- 180"""
    self.get_logger().info("Trajectory deviation: °")


def main(args=None):
    rclpy.init(args=args)

    esp_node = Esp()

    rclpy.spin(esp_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    esp_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    