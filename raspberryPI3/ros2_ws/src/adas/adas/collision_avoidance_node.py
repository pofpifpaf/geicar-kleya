import rclpy
from rclpy.node import Node


from interfaces.msg import Ultrasonic, MotorsOrder


STOP = 50
FIRST_MAX = 65

class collision_avoidance(Node):

    def __init__(self):

        super().__init__('collision_avoidance_node')
        self.publisher_motors_order = self.create_publisher(MotorsOrder, 'motors_order', 10)

        self.subscription = self.create_subscription(Ultrasonic,'us_data', self.ultrasonic_callback, 10)
        self.subscription = self.create_subscription(MotorsOrder,'motors_order_raw', self.motors_order_callback, 10)
        self.subscription  # prevent unused variable warning

        self.ultra_front_left = 300
        self.ultra_front_right = 300
        self.ultra_front_center = 300

        self.motor_right_rear_pwm = 50
        self.motor_left_rear_pwm = 50
        self.motor_steering_angle = 0

        self.get_logger().info("collision_avoidance_node READY")


    def ultrasonic_callback(self, us_data : Ultrasonic):

        self.ultra_front_left = us_data.front_left
        self.ultra_front_right = us_data.front_right
        self.ultra_front_center = us_data.front_center

        self.detect_collision()

    def motors_order_callback(self, motors_order : MotorsOrder):

        self.motor_right_rear_pwm = motors_order.right_rear_pwm
        self.motor_left_rear_pwm = motors_order.left_rear_pwm
        self.motor_steering_angle = motors_order.steering_angle

    def detect_collision(self):

        if self.motor_right_rear_pwm > STOP or self.motor_left_rear_pwm > STOP:

            if self.ultra_front_left < 20 or self.ultra_front_right < 20 or self.ultra_front_center < 20:

                self.motor_right_rear_pwm = STOP
                self.motor_left_rear_pwm = STOP
                self.get_logger().info("Detecting obstacle <20 cm: Stopping car")


            elif ((20 < self.ultra_front_left < 100) or
                  (20 < self.ultra_front_right < 100) or
                  (20 < self.ultra_front_center < 100)):

                self.motor_right_rear_pwm = min(self.motor_right_rear_pwm, FIRST_MAX)
                self.motor_left_rear_pwm = min(self.motor_left_rear_pwm, FIRST_MAX)
                self.get_logger().info("Detecting obstacle: Speed limit 30%")




        msg = MotorsOrder()
        msg.right_rear_pwm = self.motor_right_rear_pwm
        msg.left_rear_pwm = self.motor_left_rear_pwm
        msg.steering_angle = self.motor_steering_angle

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