import rclpy
from rclpy.node import Node
import math

from interfaces.msg import MotorsOrder, JoystickOrder, MotorsFeedback, MotorsOrderAdas
from std_srvs.srv import SetBool  #Service pour ON/OFF 

# Global Variable
SPEED_TOLERANCE = 0.05  #Tolérance vitesse 5%
SPEED_CORRECTION = 2  #Gain de correction (à ajuster si besoin)

STATE_NOTHING = "state_nothing"
STATE_MODE_OFF = "state_mode_off"
STATE_NOT_OK = "state_not_ok"
STATE_OK = "state_ok"

REFRESH_PERIOD = 0.002  #2 ms


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
        else:
            self.auto_speed = False
            self.motor_right_rear_pwm_offset = 0
            self.motor_left_rear_pwm_offset = 0
            self.get_logger().info("ACC OFF")

        response.success = True
        response.message = "OK"
        return response

    ####################################
    ######## Speed Implementation ######
    ####################################

    def regulate_speed(self):
        """
        Régulation avec tolérance +-5 %
        si vitesse trop basse/haute que tolerance corrige 
        """

        #pas en mode auto on fait rien
        if not self.auto_speed:
            self.state = STATE_MODE_OFF
            if self.state != self.prev_state:
                self.get_logger().info("Speed regulation OFF")
            self.prev_state = self.state
            return

        self.state = STATE_NOTHING
        self.active = False

        #Calcul de la vitesse actuelle
        actual_speed = (self.left_rear_speed + self.right_rear_speed) / 2.0
        delta = self.target_speed - actual_speed
        tolerance = SPEED_TOLERANCE * abs(self.target_speed)

        #Vitesse ok, pas de correction
        if abs(delta) < tolerance:
            self.state = STATE_OK
            if self.state != self.prev_state:
                self.get_logger().info("Speed within tolerance")

        #Vitesse trop haute/basse, correction
        else:
            pwm = SPEED_CORRECTION * delta

            self.motor_right_rear_pwm_offset = int(pwm)
            self.motor_left_rear_pwm_offset = int(pwm)
            self.active = True

            self.state = STATE_NOT_OK
            if self.state != self.prev_state:
                self.get_logger().info("Correcting speed")

        #Publishing
        if self.prev_state != self.state:
            msg = MotorsOrderAdas()
            msg.offset_right_rear_pwm = self.motor_right_rear_pwm_offset
            msg.offset_left_rear_pwm = self.motor_left_rear_pwm_offset
            msg.offset_steering_angle = self.steering_angle_offset

            msg.max_pwm = self.max_pwm
            msg.emergency_stop = self.emergency_stop
            msg.active = self.active

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
