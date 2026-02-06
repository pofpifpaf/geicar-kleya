import rclpy
from rclpy.node import Node

from interfaces.msg import Ultrasonic, MotorsOrder, MotorsOrderAdas

from rcl_interfaces.msg import SetParametersResult

STOP = 50

STATE_STOP = "state_stop"
STATE_STOP_REAR = "state_stop_rear"
STATE_NOTHING = "state_nothing"
STATE_SLOW = "state_slow"

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

        self.ultra_rear_left = 300
        self.ultra_rear_right = 300
        self.ultra_rear_center = 300

        self.motor_right_rear_pwm = 0
        self.motor_left_rear_pwm = 0

        self.emergency_stop = False

        self.state = STATE_NOTHING
        self.prev_state = STATE_NOTHING

        self.active = False

        self.declare_parameter("threshold_stop", 20)
        self.declare_parameter("threshold_slow", 40)
        self.declare_parameter("threshold_rear", 30)

        self.declare_parameter("slow_speed_percentage", 30)

        self.threshold_stop = self.get_parameter("threshold_stop").value
        self.threshold_slow = self.get_parameter("threshold_slow").value
        self.threshold_rear = self.get_parameter("threshold_rear").value

        self.slow_speed_percentage = self.get_parameter("slow_speed_percentage").value

        self.add_on_set_parameters_callback(self.param_callback)

        self.timer = self.create_timer(REFRESH_PERIOD, self.detect_collision)

        self.get_logger().info("collision_avoidance_node READY")

    # Parameters callback
    def param_callback(self, params):

        for p in params:

            if (p.name == "threshold_slow"):

                self.threshold_slow = p.value
                self.get_logger().info(f"Parameter {p.name} changed to {self.threshold_slow}")

            if (p.name == "threshold_stop"):

                self.threshold_stop = p.value
                self.get_logger().info(f"Parameter {p.name} changed to {self.threshold_stop}")

            if (p.name == "slow_speed_percentage"):

                self.slow_speed_percentage = p.value
                self.get_logger().info(f"Parameter {p.name} changed to {self.slow_speed_percentage}")

            if (p.name == "threshold_rear"):

                self.threshold_rear = p.value
                self.get_logger().info(f"Parameter {p.name} changed to {self.threshold_rear}")

        return SetParametersResult(successful=True)

    def ultrasonic_callback(self, us_data : Ultrasonic):

        self.ultra_front_left = us_data.front_left
        self.ultra_front_right = us_data.front_right
        self.ultra_front_center = us_data.front_center

        self.ultra_rear_left = us_data.rear_left
        self.ultra_rear_right = us_data.rear_right
        self.ultra_rear_center = us_data.rear_center

    def motors_order_raw_callback(self, msg):

        self.motor_right_rear_pwm = msg.right_rear_pwm
        self.motor_left_rear_pwm = msg.left_rear_pwm

    def detect_collision(self):

        self.active = False
        self.state = STATE_NOTHING
        self.emergency_stop = False
        max_pwm = 50
    
        if ((self.motor_right_rear_pwm >= STOP 
             or self.motor_left_rear_pwm >= STOP) and 
            (self.ultra_front_left < self.threshold_stop or 
            self.ultra_front_right < self.threshold_stop or 
            self.ultra_front_center < self.threshold_stop)):

            self.emergency_stop = True
            self.active = True

            self.state = STATE_STOP

            if self.state != self.prev_state and self.active:
                self.get_logger().info(f"Detecting obstacle <{self.threshold_stop}cm ahead of car : Stopping car")


        elif ((self.motor_right_rear_pwm >= STOP 
             or self.motor_left_rear_pwm >= STOP) and 
             (self.threshold_stop < self.ultra_front_center < self.threshold_slow)):
        
            max_pwm = self.slow_speed_percentage/2
            self.active = True
            
            self.state = STATE_SLOW

            if self.state != self.prev_state and self.active:
                self.get_logger().info(f"Detecting obstacle <{self.threshold_slow}cm : Speed limit {self.slow_speed_percentage}%")

        elif ((self.motor_right_rear_pwm < STOP 
             or self.motor_left_rear_pwm < STOP) and 
            (self.ultra_rear_left < self.threshold_rear or 
            self.ultra_rear_right < self.threshold_rear or 
            self.ultra_rear_center < self.threshold_rear)):
                
            self.emergency_stop = True
            self.active = True
            
            self.state = STATE_STOP_REAR

            if self.state != self.prev_state and self.active:
                self.get_logger().info(f"Detecting obstacle <{self.threshold_rear}cm behind car : Stopping car")

        
        # Publishing
        if self.active or (self.state != self.prev_state):

            msg = MotorsOrderAdas()

            msg.max_pwm = int(max_pwm)
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