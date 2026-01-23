import rclpy
from rclpy.node import Node
import math

from interfaces.msg import MotorsOrder, JoystickOrder, MotorsFeedback, MotorsOrderAdas, Ultrasonic
from std_srvs.srv import SetBool  # Service pour ON/OFF

# Global Variable
SPEED_TOLERANCE = 0.05  # Tolérance vitesse 5%
STOP = 50
MAX_PWM = 100


DETECT_DISTANCE_CM = 100   # < 1 m
STOP_DETECTING_DISTANCE_CM = 150


SAFE_MIN_CM = 60           # 80 cm
SAFE_MAX_CM = 100          # 100 cm

N = 5

# States
STATE_NOTHING = "state_nothing"
STATE_MANUAL_MODE = "state_manual_mode"
STATE_AUTOMATIC_MODE_CORRECTING = "state_automatic_mode_correcting"
STATE_AUTOMATIC_MODE_NO_CORRECTING = "state_automatic_mode_no_correcting"

# New states for distance keeping
# STATE_VEHICLE_DETECTED = "state_vehicle_detected_<1m"
# STATE_KEEP_DISTANCE = "state_keep_distance_80<x>100cm"
# STATE_EMERGENCY_STOP_FRONT = "state_emergency_stop_front"

STATE_DISTANCE_80CM_1M = "state_distance_80cm_1m"
STATE_DISTANCE_80CM_LESS = "state_distance_80cm_less"
STATE_DISTANCE_1M_PLUS = "state_distance_1m_plus"

REFRESH_PERIOD = 0.01  # 10 ms
MIN_TOLERANCE = 1.0 


class SpeedRegulation(Node):
    def __init__(self):
        # Initialization of the node
        super().__init__('acc_node')

        self.publisher_motors_order = self.create_publisher(MotorsOrderAdas, 'motors_order_acc', 10)

        # Subscriptions
        self.subscription = self.create_subscription(MotorsOrder, 'motors_order_raw', self.motors_order_callback, 10)
        self.subscription = self.create_subscription(MotorsFeedback, 'motors_feedback', self.motors_feedback_callback, 10)
        self.create_subscription(Ultrasonic, 'us_data', self.ultrasonic_callback, 10)
        self.subscription  # prevent unused variable warning

        # Speed regulation variables
        self.target_speed = 0.0
        self.target_PWM = 0
        self.auto_mode_on = False

        # Motors order
        self.motor_right_rear_pwm = 0
        self.motor_left_rear_pwm = 0
        self.motor_steering_angle = 0

        # Ultrasonic
        self.ultra_front_center = 300
        self.vehicle_detected = False

        # Motors feedback
        self.left_rear_speed = 0.0
        self.right_rear_speed = 0.0

        self.command_left_rear_pwm = STOP
        self.command_right_rear_pwm = STOP
        self.command_pwm = False

        self.last_pwm_command = STOP 

        self.max_pwm = 50  # max_pwm relatif

        self.emergency_stop = False
        self.active = False
        self.state = STATE_NOTHING
        self.prev_state = STATE_NOTHING

        # Service (appeler dans joystick_to_cmd)
        self.srv_ACC = self.create_service(SetBool, 'ACC', self.call_ACC_service)

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

    def ultrasonic_callback(self, us_data: Ultrasonic):
        self.ultra_front_center = us_data.front_center
        if self.ultra_front_center > STOP_DETECTING_DISTANCE_CM:
            self.vehicle_detected = False
        elif self.ultra_front_center < DETECT_DISTANCE_CM:
            self.vehicle_detected = True

    ###########################################
    ######## Service implementation ###########
    ###########################################

    def call_ACC_service(self, request, response):
        if request.data:
            self.auto_mode_on = True
            self.target_speed = (self.left_rear_speed + self.right_rear_speed) / 2.0
            self.target_PWM = (self.motor_left_rear_pwm + self.motor_right_rear_pwm) / 2.0
            self.get_logger().info(
                f"ACC ON : target speed={self.target_speed:.2f} and target PWM={self.target_PWM:.2f})"
            )
        else:
            self.auto_mode_on = False
            self.command_left_rear_pwm = STOP
            self.command_right_rear_pwm = STOP

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

        self.state = STATE_NOTHING
        self.active = False

        self.command_right_rear_pwm = STOP
        self.command_left_rear_pwm = STOP
        self.command_pwm = False

        # MODE OFF (MANUEL)
        if not self.auto_mode_on:
            self.state = STATE_MANUAL_MODE
            if self.state != self.prev_state:
                self.get_logger().info("ACC OFF")
                msg = MotorsOrderAdas()        
                msg.active = self.active
                msg.state = self.state
                self.publisher_motors_order.publish(msg)
            self.prev_state = self.state
            return

        # MODE ON (AUTO)
        self.command_pwm = True
        

        # DETECTION VEHICULE < 1m
        distance = self.ultra_front_center

        if self.vehicle_detected:
            self.active = True
            if distance < SAFE_MIN_CM:
                self.state = STATE_DISTANCE_80CM_LESS
                self.command_left_rear_pwm = int(max(self.last_pwm_command - N, STOP))
                self.command_right_rear_pwm = int(max(self.last_pwm_command - N, STOP))
                if self.state != self.prev_state:
                    self.get_logger().info(f"ACC: Too close ({distance}cm) -> slow down)")
            
            elif distance > SAFE_MAX_CM:
                self.state = STATE_DISTANCE_1M_PLUS
                self.command_left_rear_pwm = int(min(self.last_pwm_command + N, MAX_PWM))
                self.command_right_rear_pwm = int(min(self.last_pwm_command + N, MAX_PWM))
                if self.state != self.prev_state:
                    self.get_logger().info(f"ACC: Too far ({distance}cm) -> speed up)")

            else:
                self.state = STATE_DISTANCE_80CM_1M
                self.command_left_rear_pwm = int(self.last_pwm_command)
                self.command_right_rear_pwm = int(self.last_pwm_command)
                if self.state != self.prev_state:
                    self.get_logger().info(f"ACC: Correct distance ({distance}cm)")
                

        

        else: 
            user_PWM = (self.motor_left_rear_pwm + self.motor_right_rear_pwm) / 2.0
            pwm_offset = self.target_PWM - user_PWM

            if pwm_offset < 0 or user_PWM < STOP:
                self.state = STATE_AUTOMATIC_MODE_NO_CORRECTING
                if self.state != self.prev_state:
                    self.get_logger().info("ACC ON: user controlling beyond desired speed")
            else:
                self.state = STATE_AUTOMATIC_MODE_CORRECTING
                self.active = True
                

                self.command_left_rear_pwm = int(self.target_PWM)
                self.command_right_rear_pwm = int(self.target_PWM)

                if self.state != self.prev_state:
                    self.get_logger().info("ACC: Speed out of tolerance -> Correcting speed")

        

        # Publishing
        if (self.prev_state != self.state) or self.last_pwm_command != int((self.command_left_rear_pwm + self.command_right_rear_pwm) / 2.0):
            msg = MotorsOrderAdas()
            msg.command_left_rear_pwm = self.command_left_rear_pwm
            msg.command_right_rear_pwm = self.command_right_rear_pwm
            msg.command_pwm = self.command_pwm
            msg.active = self.active
            msg.state = self.state
            self.publisher_motors_order.publish(msg)

        self.prev_state = self.state
        self.last_pwm_command = (self.command_left_rear_pwm + self.command_right_rear_pwm) / 2.0
        return

def main(args=None):
    rclpy.init(args=args)

    speed_regulation = SpeedRegulation()
    rclpy.spin(speed_regulation)

    speed_regulation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()