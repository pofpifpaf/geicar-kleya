import rclpy
from rclpy.node import Node

from interfaces.msg import Ultrasonic, MotorsOrderAdas

STOP = 50
FIRST_MAX = 65

class collision_avoidance(Node):

    def __init__(self):

        super().__init__('collision_avoidance_node')
        self.publisher_motors_order = self.create_publisher(MotorsOrderAdas, 'motors_order_collision', 10)

        self.subscription = self.create_subscription(Ultrasonic,'us_data', self.ultrasonic_callback, 10)
        self.subscription  # prevent unused variable warning

        self.ultra_front_left = 300
        self.ultra_front_right = 300
        self.ultra_front_center = 300

        self.motor_right_rear_pwm_offset = 0
        self.motor_left_rear_pwm_offset = 0
        self.motor_steering_angle_offset = 0

        self.max_pwm = 100
        self.emergency_stop = False

        self.active = False

        self.get_logger().info("collision_avoidance_node READY")


    def ultrasonic_callback(self, us_data : Ultrasonic):

        self.ultra_front_left = us_data.front_left
        self.ultra_front_right = us_data.front_right
        self.ultra_front_center = us_data.front_center

        self.detect_collision()

    def detect_collision(self):

        self.active = False
        self.emergency_stop = False

        if self.motor_right_rear_pwm > STOP or self.motor_left_rear_pwm > STOP:

            if self.ultra_front_left < 20 or self.ultra_front_right < 20 or self.ultra_front_center < 20:

                self.emergency_stop = True
                self.get_logger().info("Detecting obstacle <20 cm: Stopping car")
                self.active = True

            elif ((20 < self.ultra_front_left < 100) or
                  (20 < self.ultra_front_right < 100) or
                  (20 < self.ultra_front_center < 100)):

                self.max_pwm = 30
                self.get_logger().info("Detecting obstacle: Speed limit 30%")
                self.active = True


        # Publishing
        msg = MotorsOrderAdas()

        msg.offset_right_rear_pwm = self.motor_right_rear_pwm_offset
        msg.offset_left_rear_pwm = self.motor_left_rear_pwm_offset

        msg.offset_steering_angle = self.motor_steering_angle_offset

        msg.max_pwm = self.max_pwm

        msg.emergency_stop = self.emergency_stop

        msg.changes = self.active

        self.publisher_motors_order.publish(msg)




def main(args=None):
    rclpy.init(args=args)

    collision_avoidance_node = collision_avoidance()

    rclpy.spin(collision_avoidance_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    collision_avoidance_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()