import rclpy
from rclpy.node import Node
import math

from interfaces.msg import MotorsOrder, JoystickOrder, MotorsFeedback, MotorsOrderAdas
from std_srvs.srv import SetBool  #Service pour ON/OFF 

# Global Variable
SPEED_TOLERANCE = 0.05  #Tolérance vitesse 5%
SPEED_CORRECTION = 3  #Gain de correction (à ajuster si besoin)

STATE_NOTHING = "state_nothing"
STATE_MODE_OFF = "state_mode_off"
STATE_NOT_OK = "state_not_ok"
STATE_OK = "state_ok"

REFRESH_PERIOD = 0.01  #10ms


class speed_regulation(Node):
    def __init__(self):
        # Initialization of the node
        super().__init__('ACC_node')

        self.publisher_motors_order = self.create_publisher(MotorsOrderAdas, 'motors_order_ACC', 10)

        # Subscriptions
        self.subscription = self.create_subscription(MotorsOrder, 'motors_order_raw', self.motors_order_callback, 10)
        self.subscription = self.create_subscription(MotorsFeedback, 'motors_feedback', self.motors_feedback_callback, 10)
        self.subscription  # prevent unused variable warning

        #Speed regulation variables
        self.target_speed = 0.0
        self.auto_speed = False  
        self.prev_correcting = False

        # Motors order
        self.motor_right_rear_pwm = 0
        self.motor_left_rear_pwm = 0
        self.motor_steering_angle = 0

        # Motors feedback
        self.left_rear_speed = 0.0
        self.right_rear_speed = 0.0

        self.motor_right_rear_pwm_offset = 0
        self.motor_left_rear_pwm_offset = 0
        self.steering_angle_offset = 0

        self.max_pwm = 50  # max_pwm relatif

        self.emergency_stop = False
        self.active = False
        self.state = STATE_NOTHING
        self.prev_state = STATE_NOTHING

        # Service (appeler dans joystick_to_cmd)
        self.srv_ACC = self.create_service(SetBool,'ACC',self.call_ACC_service)

        self.timer = self.create_timer(REFRESH_PERIOD, self.regulate_speed)

        self.get_logger().info("ACC_node READY")

    ####################################
    ######### Callback Section #########
    ####################################

    def motors_order_callback(self, motors_order: MotorsOrder):
        self.motor_right_rear_pwm = motors_order.right_rear_pwm
        self.motor_left_rear_pwm = motors_order.left_rear_pwm
        self.motor_steering_angle = motors_order.steering_angle

    def motors_feedback_callback(self, msg: MotorsFeedback):
        self.left_rear_speed = msg.left_rear_speed
        self.right_rear_speed = msg.right_rear_speed

    ###########################################
    ######## Service implementation ###########
    ###########################################

    def call_ACC_service(self, request, response):
        if request.data:
            self.auto_speed = True
            self.target_speed = (self.left_rear_speed + self.right_rear_speed) / 2.0
            self.get_logger().info(f"ACC ON : target speed={self.target_speed:.2f})")

            # reset offsets on engage
            self.motor_right_rear_pwm_offset = 0
            self.motor_left_rear_pwm_offset = 0
            self.active = False

        else:
            self.auto_speed = False
            self.motor_right_rear_pwm_offset = 0
            self.motor_left_rear_pwm_offset = 0
            self.active = False
            self.get_logger().info("ACC OFF")

            # publish  once when turning OFF
            msg = MotorsOrderAdas()
            msg.offset_right_rear_pwm = self.motor_right_rear_pwm_offset
            msg.offset_left_rear_pwm = self.motor_left_rear_pwm_offset
            msg.offset_steering_angle = self.steering_angle_offset
            msg.max_pwm = self.max_pwm
            msg.emergency_stop = self.emergency_stop
            msg.active = self.active
            msg.state = self.state

            self.publisher_motors_order.publish(msg)

        response.success = True
        response.message = "OK"
        return response

    ####################################
    ######## Speed Implementation ######
    ####################################
    def regulate_speed(self):
        if not self.auto_speed:
            self.state = STATE_MODE_OFF
             # force offsets to zero
            self.motor_right_rear_pwm_offset = 0
            self.motor_left_rear_pwm_offset = 0

        # publish every cycle while OFF 
            msg = MotorsOrderAdas()
            msg.offset_right_rear_pwm = 0
            msg.offset_left_rear_pwm = 0
            msg.offset_steering_angle = self.steering_angle_offset
            msg.max_pwm = self.max_pwm
            msg.emergency_stop = self.emergency_stop
            msg.active = False
            msg.state = self.state
            self.publisher_motors_order.publish(msg)
            if self.state != self.prev_state:
                self.get_logger().info("Speed regulation OFF")
            self.prev_state = self.state
            return

            # ACC engagé tant que auto_speed = True
        self.active = True

        actual_speed = (self.left_rear_speed + self.right_rear_speed) / 2.0
        delta = self.target_speed - actual_speed
        tolerance = SPEED_TOLERANCE * abs(self.target_speed)

        user_accelerating = (self.motor_left_rear_pwm > 50) or (self.motor_right_rear_pwm > 50)

        if abs(delta) < tolerance:
            self.motor_right_rear_pwm_offset = 0
            self.motor_left_rear_pwm_offset = 0
        else:
            pwm = SPEED_CORRECTION * delta

            # autoriser l’override conducteur : ne pas freiner pendant qu’il accélère
            if user_accelerating and pwm < 0:
                pwm = 0

            pwm = max(min(pwm, self.max_pwm), -self.max_pwm)
            self.motor_right_rear_pwm_offset = int(round(pwm))
            self.motor_left_rear_pwm_offset  = int(round(pwm))

        correcting_log = (abs(delta) >= tolerance)

        if correcting_log != self.prev_correcting:
            if correcting_log:
                self.get_logger().info("Correcting speed")
            else:
                self.get_logger().info("Speed within tolerance")

        self.prev_correcting = correcting_log

        correcting = (self.motor_right_rear_pwm_offset != 0 or self.motor_left_rear_pwm_offset != 0)
        self.state = STATE_NOT_OK if correcting else STATE_OK

        msg = MotorsOrderAdas()
        msg.offset_right_rear_pwm = self.motor_right_rear_pwm_offset
        msg.offset_left_rear_pwm = self.motor_left_rear_pwm_offset
        msg.offset_steering_angle = self.steering_angle_offset
        msg.max_pwm = self.max_pwm
        msg.emergency_stop = self.emergency_stop
        msg.active = self.active
        msg.state = self.state
        self.publisher_motors_order.publish(msg)

        self.prev_state = self.state



def main(args=None):
    rclpy.init(args=args)

    speed_regulation_node = speed_regulation()

    rclpy.spin(speed_regulation_node)

    speed_regulation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
