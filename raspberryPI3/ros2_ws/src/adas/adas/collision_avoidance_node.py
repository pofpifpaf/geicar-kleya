import rclpy
from rclpy.node import Node

from interfaces.msg import Ultrasonic, MotorsOrder, MotorsOrderAdas

STOP = 50
FIRST_MAX = 65

STATE_20CM = "state_20cm"
STATE_NOTHING = "state_nothing"
STATE_1M = "state_1m"

REFRESH_PERIOD = 0.002

class collision_avoidance(Node):

    def __init__(self):

        super().__init__('collision_avoidance_node')
        self.publisher_motors_order = self.create_publisher(MotorsOrderAdas, 'motors_order_collision', 10)

        self.subscription = self.create_subscription(Ultrasonic,'us_data', self.ultrasonic_callback, 10)
        self.subscription = self.create_subscription(MotorsOrder,'motors_order_raw', self.motors_order_raw_callback, 10)
        self.subscription  # prevent unused variable warning

        self.ultra_front_left = 300
        self.ultra_front_right = 300
        self.ultra_front_center = 300

        self.motor_right_rear_pwm_offset = 0
        self.motor_left_rear_pwm_offset = 0
        self.steering_angle_offset = 0

        self.motor_right_rear_pwm = 0
        self.motor_left_rear_pwm = 0
        self.steering_angle = 0

        self.max_pwm = 100
        self.emergency_stop = False

        self.state = STATE_NOTHING
        self.prev_state = STATE_NOTHING

        self.active = False

        self.timer = self.create_timer(REFRESH_PERIOD, self.detect_collision)

        self.get_logger().info("collision_avoidance_node READY")


    def ultrasonic_callback(self, us_data : Ultrasonic):

        self.ultra_front_left = us_data.front_left
        self.ultra_front_right = us_data.front_right
        self.ultra_front_center = us_data.front_center

        self.detect_collision()

    def motors_order_raw_callback(self, msg):

        self.motor_right_rear_pwm = msg.right_rear_pwm
        self.motor_left_rear_pwm = msg.left_rear_pwm

        self.motor_steering_angle = msg.steering_angle

    def detect_collision(self):

        self.active = False
        self.state = STATE_NOTHING
        self.emergency_stop = False
        self.max_pwm = 50

    
        if self.ultra_front_left < 20 or self.ultra_front_right < 20 or self.ultra_front_center < 20:

            if self.motor_right_rear_pwm > STOP or self.motor_left_rear_pwm > STOP:

                self.emergency_stop = True
                self.active = True
                
            self.state = STATE_20CM

            if self.state != self.prev_state:
                self.get_logger().info("Detecting obstacle <20 cm: Stopping car")


        elif ((20 < self.ultra_front_left < 100) or
              (20 < self.ultra_front_right < 100) or
              (20 < self.ultra_front_center < 100)):
        
            self.max_pwm = 15
            self.active = True
            
            self.state = STATE_1M

            if self.state != self.prev_state:
                self.get_logger().info("Detecting obstacle: Speed limit 30%")
                changes = True


        # Publishing
        if self.state != self.prev_state:
            msg = MotorsOrderAdas()

            msg.offset_right_rear_pwm = self.motor_right_rear_pwm_offset
            msg.offset_left_rear_pwm = self.motor_left_rear_pwm_offset

            msg.offset_steering_angle = self.steering_angle_offset

            msg.max_pwm = self.max_pwm

            msg.emergency_stop = self.emergency_stop

            msg.state = self.state

            msg.active = self.active

            self.publisher_motors_order.publish(msg)

        self.prev_state = self.state




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