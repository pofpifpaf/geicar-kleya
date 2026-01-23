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
SAFE_MIN_CM = 80           # 80 cm
SAFE_MAX_CM = 100          # 100 cm
   

# States
STATE_NOTHING = "state_nothing"
STATE_MANUAL_MODE = "state_manual_mode"
STATE_AUTOMATIC_MODE_CORRECTING = "state_automatic_mode_correcting"
STATE_AUTOMATIC_MODE_NO_CORRECTING = "state_automatic_mode_no_correcting"

# New states for distance keeping
STATE_KEEP_DISTANCE = "state_keep_distance_80<x>100cm"


REFRESH_PERIOD = 0.01  # 10 ms
MIN_TOLERANCE = 1.0 

# Control gains
KP_DISTANCE = 0.5      # distancia (cm → PWM)
KD_DISTANCE = 3.0      # vitesse relative (cm/s → PWM)



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

        # Ultrasonic
        self.ultra_front_center = 300
        self.last_distance = self.ultra_front_center

        # Motors feedback
        self.left_rear_speed = 0.0
        self.right_rear_speed = 0.0

        self.motor_right_rear_pwm_offset = 0
        self.motor_left_rear_pwm_offset = 0

        self.last_pwm_offset = (self.motor_left_rear_pwm_offset + self.motor_right_rear_pwm_offset) / 2.0

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
            self.motor_right_rear_pwm_offset = 0
            self.motor_left_rear_pwm_offset = 0

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
        self.motor_left_rear_pwm_offset = 0
        self.motor_right_rear_pwm_offset = 0

        # MODE OFF (MANUEL)
        if not self.auto_mode_on:
            self.state = STATE_MANUAL_MODE
            if self.state != self.prev_state:
                self.get_logger().info("ACC OFF")
                msg = MotorsOrderAdas()
                msg.offset_right_rear_pwm = self.motor_right_rear_pwm_offset
                msg.offset_left_rear_pwm = self.motor_left_rear_pwm_offset
                msg.active = self.active
                msg.state = self.state
                self.publisher_motors_order.publish(msg)
            self.prev_state = self.state
            return

        # MODE ON (AUTO)
        distance = self.ultra_front_center

        user_PWM = (self.motor_left_rear_pwm + self.motor_right_rear_pwm) / 2.0

        if user_PWM < STOP:
            self.state = STATE_AUTOMATIC_MODE_CORRECTING
            if self.state != self.prev_state:
                self.get_logger().info("ACC ON: user controlling below STOP")

        elif user_PWM > self.target_PWM:
            self.state = STATE_AUTOMATIC_MODE_NO_CORRECTING
            if self.state != self.prev_state:
                self.get_logger().info("ACC ON: user controlling beyond desired speed")

        elif distance > DETECT_DISTANCE_CM:
            actual_speed = (self.left_rear_speed + self.right_rear_speed) / 2.0

            delta = self.target_speed - actual_speed
            tolerance = max(MIN_TOLERANCE, SPEED_TOLERANCE * abs(self.target_speed))

            if abs(delta) <= tolerance:
                self.state = STATE_AUTOMATIC_MODE_NO_CORRECTING
                if self.state != self.prev_state:
                    self.get_logger().info("Speed within tolerance")

            else:
                pwm_offset = self.target_PWM - user_PWM
                self.state = STATE_AUTOMATIC_MODE_CORRECTING
                self.active = True

                self.motor_right_rear_pwm_offset = int(pwm_offset)
                self.motor_left_rear_pwm_offset = int(pwm_offset)

                if self.state != self.prev_state:
                    self.get_logger().info("Correcting speed")

        else:
            self.state = STATE_KEEP_DISTANCE
            self.active = True
            if self.state != self.prev_state:
                    self.get_logger().info(f"Keeping distance. Current distance = {distance}")


            distance_diff = distance - self.last_distance
            relative_speed = distance_diff / REFRESH_PERIOD  # cm/s

            self.last_distance = distance

            target_distance = (SAFE_MIN_CM + SAFE_MAX_CM) / 2.0
            distance_error = distance - target_distance

            

            correction = (KP_DISTANCE * distance_error) - (KD_DISTANCE * relative_speed)

            offset = int(round(correction))

            desired = int(round(offset))

            if desired < prev - MAX_BRAKE_STEP:
                desired = prev - MAX_BRAKE_STEP
            elif desired > prev + MAX_BRAKE_STEP:
                desired = prev + MAX_BRAKE_STEP

            # anti-recul
            min_offset = int(round(STOP - user_PWM))
            if desired < min_offset:
                desired = min_offset
            
            max_offset = int(round(MAX_PWM - user_PWM))
            if derised > max_offset:
                desired = max_offset


            

            self.motor_right_rear_pwm_offset = int(desired)
            self.motor_left_rear_pwm_offset = int(desired)

        # Publishing
        if (self.prev_state != self.state) or self.last_pwm_offset != int((self.motor_left_rear_pwm_offset + self.motor_right_rear_pwm_offset) / 2.0):
            msg = MotorsOrderAdas()
            msg.offset_right_rear_pwm = self.motor_right_rear_pwm_offset
            msg.offset_left_rear_pwm = self.motor_left_rear_pwm_offset
            msg.active = self.active
            msg.state = self.state
            self.publisher_motors_order.publish(msg)

        self.prev_state = self.state
        self.last_pwm_offset = (self.motor_left_rear_pwm_offset + self.motor_right_rear_pwm_offset) / 2.0
        return


def main(args=None):
    rclpy.init(args=args)

    speed_regulation = SpeedRegulation()
    rclpy.spin(speed_regulation)

    speed_regulation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
