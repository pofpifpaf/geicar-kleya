import rclpy
from rclpy.node import Node
import math

from interfaces.msg import MotorsOrder, JoystickOrder, MotorsFeedback, MotorsOrderAdas, Ultrasonic
from std_srvs.srv import SetBool  # Service pour ON/OFF

# Global Variable
SPEED_TOLERANCE = 0.05  # Tolérance vitesse 5%
STOP = 50
DETECT_DISTANCE_CM = 100   # < 1 m
SAFE_MIN_CM = 80           # 80 cm
SAFE_MAX_CM = 100          # 100 cm

# Gains freinage
BRAKE_GAIN_SOFT = 0.4     # quand 80-100
BRAKE_GAIN_STRONG = 1.0   # quand <80

# limitation de variation de freinage (évite STOP instant)
MAX_BRAKE_STEP = 2      

# States
STATE_NOTHING = "state_nothing"
STATE_MANUAL_MODE = "state_manual_mode"
STATE_AUTOMATIC_MODE_CORRECTING = "state_automatic_mode_correcting"
STATE_AUTOMATIC_MODE_NO_CORRECTING = "state_automatic_mode_no_correcting"

# New states for distance keeping
STATE_VEHICLE_DETECTED = "state_vehicle_detected_<1m"
STATE_KEEP_DISTANCE = "state_keep_distance_80<x>100cm"
STATE_EMERGENCY_STOP_FRONT = "state_emergency_stop_front"

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

        # Motors feedback
        self.left_rear_speed = 0.0
        self.right_rear_speed = 0.0

        self.motor_right_rear_pwm_offset = 0
        self.motor_left_rear_pwm_offset = 0
        self.steering_angle_offset = 0

        self.last_pwm_offset = (self.motor_left_rear_pwm_offset + self.motor_right_rear_pwm_offset) / 2.0

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
        self.emergency_stop = False
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
                msg.offset_steering_angle = self.steering_angle_offset
                msg.max_pwm = self.max_pwm
                msg.emergency_stop = self.emergency_stop
                msg.active = self.active
                msg.state = self.state
                self.publisher_motors_order.publish(msg)
            self.prev_state = self.state
            return

        # MODE ON (AUTO)

        # DETECTION VEHICULE < 1m
        distance = self.ultra_front_center

        if distance < DETECT_DISTANCE_CM:

            self.state = STATE_VEHICLE_DETECTED
            self.active = True

            user_PWM = (self.motor_left_rear_pwm + self.motor_right_rear_pwm) / 2.0

            # Si <80cm : freinage plus fort pour remonter vers 80
            if distance < SAFE_MIN_CM:

                self.state = STATE_KEEP_DISTANCE

                error_cm = float(SAFE_MIN_CM - distance)
                brake = int(round(BRAKE_GAIN_STRONG * error_cm))

                offset = -brake

                # anti-recul 
                min_offset = int(round(STOP - user_PWM))
                if offset < min_offset:
                    offset = min_offset

                # si on est déjà à STOP et toujours <1m, on reste arrêté
                if user_PWM <= STOP:
                    offset = min_offset

                # limiter la variation d’offset (évite arrêt brutal)
                prev = int(round(self.last_pwm_offset))
                desired = int(round(offset))

                if desired < prev - MAX_BRAKE_STEP:
                    desired = prev - MAX_BRAKE_STEP
                elif desired > prev + MAX_BRAKE_STEP:
                    desired = prev + MAX_BRAKE_STEP

                if desired < min_offset:
                    desired = min_offset

                self.motor_left_rear_pwm_offset = int(desired)
                self.motor_right_rear_pwm_offset = int(desired)

                if self.state != self.prev_state:
                    self.get_logger().info(f"ACC: Too close ({distance}cm) -> slow down)")

            # Entre 80 et 100 : freinage soft pour rester dans la zone
            else:

                self.state = STATE_KEEP_DISTANCE

                error_cm = float(SAFE_MAX_CM - distance)
                brake = int(round(BRAKE_GAIN_SOFT * error_cm))

                offset = -brake

                # anti-recul 
                min_offset = int(round(STOP - user_PWM))
                if offset < min_offset:
                    offset = min_offset

                # si on est déjà à STOP et toujours <1m, on reste arrêté
                if user_PWM <= STOP:
                    offset = min_offset

                # limiter la variation d’offset
                prev = int(round(self.last_pwm_offset))
                desired = int(round(offset))

                if desired < prev - MAX_BRAKE_STEP:
                    desired = prev - MAX_BRAKE_STEP
                elif desired > prev + MAX_BRAKE_STEP:
                    desired = prev + MAX_BRAKE_STEP

                if desired < min_offset:
                    desired = min_offset

                self.motor_left_rear_pwm_offset = int(desired)
                self.motor_right_rear_pwm_offset = int(desired)

                if self.state != self.prev_state:
                    self.get_logger().info(f"ACC: Vehicle at {distance}cm -> keeping distance)")

        # Vehicle >= 1m : régulation vitesse 
        else:
            actual_speed = (self.left_rear_speed + self.right_rear_speed) / 2.0

            delta = self.target_speed - actual_speed
            tolerance = max(MIN_TOLERANCE, SPEED_TOLERANCE * abs(self.target_speed))

            if abs(delta) <= tolerance:
                self.state = STATE_AUTOMATIC_MODE_NO_CORRECTING
                if self.state != self.prev_state:
                    self.get_logger().info("Speed within tolerance")

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

                    self.motor_right_rear_pwm_offset = int(pwm_offset)
                    self.motor_left_rear_pwm_offset = int(pwm_offset)

                    if self.state != self.prev_state:
                        self.get_logger().info("Correcting speed")

        # Publishing
        if (self.prev_state != self.state) or self.last_pwm_offset != int((self.motor_left_rear_pwm_offset + self.motor_right_rear_pwm_offset) / 2.0):
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
        self.last_pwm_offset = (self.motor_left_rear_pwm_offset + self.motor_right_rear_pwm_offset) / 2.0
        return