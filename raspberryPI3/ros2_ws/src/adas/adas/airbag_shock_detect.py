import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

from interfaces.msg import MotorsOrderAdas

min_linear_acceleration = 2.5

STATE_SHOCK = "shock"
STATE_NOTHING = "nothing"

class airbag_shock_detection(Node):

    def __init__(self):
        #Initialization of the shock_detection_node Node
        super().__init__('shock_detection_node')
        self.publisher_motors_order = self.create_publisher(MotorsOrderAdas, 'motors_order_airbag', 10)

        #Subscription to the IMU topic
        self.subscription = self.create_subscription(Imu,'/imu/data', self.imu_callback, 10)
        self.subscription  # prevent unused variable warning

        self.motor_right_rear_pwm_offset = 0
        self.motor_left_rear_pwm_offset = 0
        self.motor_steering_angle_offset = 0

        self.max_pwm = 100
        self.emergency_stop = False

        self.active = False

        self.state = STATE_NOTHING
        self.prev_state = STATE_NOTHING

        self.get_logger().info("Shock detection node READY")


    # function that get called when an IMU value is received and send in the topic isShockDetect
    # true if val >= min_linear_acceleration
    def imu_callback(self, msg):

        self.emergency_stop = False
        self.active = False

        self.state = STATE_NOTHING

        # Define a variable with the minimal value for shock detection
        
        # get the data  from the imu topic
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y

        # Compare the value with the min we defined
        if (ax >= min_linear_acceleration or 
            ay >= min_linear_acceleration ):
            self.emergency_stop = True
            self.state = STATE_SHOCK
            self.active = True

            # log to debug
            self.get_logger().info(f"Accel = ({ax:.2f}, {ay:.2f}), Shock detected")

        # Publishing
        if self.prev_state != self.state:

            msg = MotorsOrderAdas()

            msg.offset_right_rear_pwm = self.motor_right_rear_pwm_offset
            msg.offset_left_rear_pwm = self.motor_left_rear_pwm_offset

            msg.offset_steering_angle = self.motor_steering_angle_offset

            msg.max_pwm = self.max_pwm

            msg.emergency_stop = self.emergency_stop

            msg.active = self.active

            self.publisher_motors_order.publish(msg)

        self.prev_state = self.state


def main(args=None):
    rclpy.init(args=args)

    shock_detection_node = airbag_shock_detection()

    rclpy.spin(shock_detection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    shock_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
