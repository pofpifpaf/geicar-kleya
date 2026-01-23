import rclpy
from rclpy.node import Node
import math

from interfaces.msg import MotorsOrder, JoystickOrder, MotorsFeedback, MotorsOrderAdas, Ultrasonic
from std_srvs.srv import SetBool  #Service pour ON/OFF 

# Global Variable
SPEED_TOLERANCE = 0.05  #Tolérance vitesse 5%
STOP = 50
MAX_PWM = 100

# Distances Ultrasonic (en cm)
DETECT_DISTANCE_CM = 100   # < 1 m
STOP_DETECTING_DISTANCE_CM = 150  # > 1.5 m
SAFE_MIN_CM = 80           # 80 cm
SAFE_MAX_CM = 100          # 100 cm


# Gains freinage/acceleration
BRAKE_GAIN_STRONG = 2.0   # quand distance < 80
ACCELERATION_GAIN = 2.0   # quand distance > 100


# States
STATE_NOTHING = "state_nothing"
STATE_MANUAL_MODE = "state_manual_mode"
STATE_AUTOMATIC_MODE_CORRECTING = "state_automatic_mode_correcting"
STATE_AUTOMATIC_MODE_NO_CORRECTING = "state_automatic_mode_no_correcting"

# New states for distance keeping
STATE_INCORRECT_DISTANCE_TO_VEHICLE = "state_incorrect_distance_to_vehicle"
STATE_CORRECT_DISTANCE_TO_VEHICLE = "state_correct_distance_to_vehicle"



# STATE_VEHICLE_DETECTED = "state_vehicle_detected_<1m"
# STATE_KEEP_DISTANCE = "state_keep_distance_80<x>100cm"
# STATE_EMERGENCY_STOP_FRONT = "state_emergency_stop_front"

REFRESH_PERIOD = 0.01  #10 ms
MIN_TOLERANCE = 1.0  #Minimum tolerance in RPM


class SpeedRegulation(Node):
    def __init__(self):
        # Initialization of the node
        super().__init__('acc_node')

        self.publisher_motors_order = self.create_publisher(MotorsOrderAdas, 'motors_order_acc', 10)

        # Subscriptions
        self.subscription = self.create_subscription(MotorsOrder, 'motors_order_raw', self.motors_order_callback, 10)
        self.subscription = self.create_subscription(MotorsFeedback, 'motors_feedback', self.motors_feedback_callback, 10)
        self.subscription = self.create_subscription(Ultrasonic, 'us_data', self.ultrasonic_callback, 10)
        self.subscription  # prevent unused variable warning

        #Speed regulation variables
        self.target_speed = 0.0
        self.target_PWM = STOP
        self.auto_mode_on = False  

        # Motors input order
        self.motor_right_rear_pwm = STOP
        self.motor_left_rear_pwm = STOP

        # Ultrasonic
        self.ultra_front_center = 300
        self.vehicle_detected = False

        # Motors feedback
        self.left_rear_speed = 0.0
        self.right_rear_speed = 0.0


        # Motors output orders
        self.command_right_rear_pwm = STOP
        self.command_left_rear_pwm = STOP

        # Last values
        self.last_pwm = int((self.command_left_rear_pwm + self.command_right_rear_pwm) /2.0)

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

    def motors_feedback_callback(self, msg: MotorsFeedback):
        self.left_rear_speed = msg.left_rear_speed
        self.right_rear_speed = msg.right_rear_speed

    def ultrasonic_callback(self, us_data : Ultrasonic):
        self.ultra_front_center = us_data.front_center
        if self.ultra_front_center < DETECT_DISTANCE_CM:
            self.vehicle_detected = True
        elif self.ultra_front_center > STOP_DETECTING_DISTANCE_CM:
            self.vehicle_detected = False


    ###########################################
    ######## Service implementation ###########
    ###########################################

    def call_ACC_service(self, request, response):
        if request.data:
            self.auto_mode_on = True
            self.target_speed = (self.left_rear_speed + self.right_rear_speed) / 2.0
            self.target_PWM = int((self.motor_left_rear_pwm + self.motor_right_rear_pwm) / 2.0)
            self.get_logger().info(f"ACC ON : target speed={self.target_speed:.2f} and target PWM={self.target_PWM:.2f})")
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
        self.command_right_rear_pwm = STOP
        self.command_left_rear_pwm = STOP

        # MODE OFF (MANUAL MODE)
        if not self.auto_mode_on:
            self.state = STATE_MANUAL_MODE
            if self.state != self.prev_state:
                self.get_logger().info("ACC OFF")
                msg = MotorsOrderAdas()
                msg.state = self.state
                self.publisher_motors_order.publish(msg)
            self.prev_state = self.state
            return
        
        
        # MODE ON (AUTOMATIC MODE)

        user_pwm = int((self.motor_left_rear_pwm + self.motor_right_rear_pwm) / 2.0)
        
        #### User exceeds target speed => No correction
        if user_pwm > self.target_PWM:
            self.state = STATE_AUTOMATIC_MODE_NO_CORRECTING
            if self.state != self.prev_state:
                self.get_logger().info("ACC ON: User controlling beyond desired speed -> no correction")
        
        #### User trying to go in reverse => No correction
        elif user_pwm < STOP:
            self.state = STATE_AUTOMATIC_MODE_NO_CORRECTING
            if self.state != self.prev_state:
                self.get_logger().info("ACC ON: User controlling in reverse -> no correction")

        else:

            ## NO VEHICLE DETECTED (Normal speed regulation)
            if not self.vehicle_detected:
                
                ### Actual speed calculation
                actual_speed = (self.left_rear_speed + self.right_rear_speed) / 2.0

                ### Speed error and tolerance calculation
                delta = self.target_speed - actual_speed
                tolerance = max(MIN_TOLERANCE, SPEED_TOLERANCE * abs(self.target_speed))

                ### Speed in tolerance => No correction
                if abs(delta) <= tolerance:
                    self.state = STATE_AUTOMATIC_MODE_NO_CORRECTING
                    if self.state != self.prev_state:
                        self.get_logger().info("ACC ON: Speed within tolerance -> no correction")

                ### Speed out of tolerance => Correction
                else:
                    self.state = STATE_AUTOMATIC_MODE_CORRECTING
                    if self.state != self.prev_state:
                        self.get_logger().info("ACC ON: Correcting speed")
                    
                    self.active = True
                    self.command_right_rear_pwm = self.target_PWM
                    self.command_left_rear_pwm = self.target_PWM



            ## VEHICLE DETECTED AHEAD (Distance keeping mode)
            else:
                distance = self.ultra_front_center
                ### If distance < 80 cm => slow down
                if distance < SAFE_MIN_CM:
                    self.state = STATE_INCORRECT_DISTANCE_TO_VEHICLE
                    if self.state != self.prev_state:
                        self.get_logger().info(f"ACC ON: Too close ({distance}cm) -> slow down)")
                    
                    self.active = True

                    error_cm = float(SAFE_MIN_CM - distance)      # 80-60 = 20
                    brake = int(round(BRAKE_GAIN_STRONG * error_cm))

                    if self.last_pwm - brake <= STOP:
                        self.command_left_rear_pwm = STOP
                        self.command_right_rear_pwm = STOP
                    else:
                        self.command_left_rear_pwm = self.last_pwm - brake
                        self.command_right_rear_pwm = self.last_pwm - brake

                ### If distance > 100 cm => speed up
                elif distance > SAFE_MAX_CM:
                    self.state = STATE_INCORRECT_DISTANCE_TO_VEHICLE
                    if self.state != self.prev_state:
                        self.get_logger().info(f"ACC ON: Vehicle far away ({distance}cm) -> speed up)")
                    
                    self.active = True

                    error_cm = float(distance - SAFE_MAX_CM)      # 120-100 = 20
                    acceleration = int(round(ACCELERATION_GAIN * error_cm))

                    if self.last_pwm + acceleration >= MAX_PWM:
                        self.command_left_rear_pwm = MAX_PWM
                        self.command_right_rear_pwm = MAX_PWM
                    else:
                        self.command_left_rear_pwm = self.last_pwm + acceleration
                        self.command_right_rear_pwm = self.last_pwm + acceleration

                ### Correct distance => maintain user PWM
                else:
                    self.state = STATE_CORRECT_DISTANCE_TO_VEHICLE
                    if self.state != self.prev_state:
                        self.get_logger().info(f"ACC ON: Correct distance to vehicle ({distance}cm) -> maintaining distance)")
                    self.command_left_rear_pwm = self.last_pwm
                    self.command_right_rear_pwm = self.last_pwm
                
        #Publishing
        if self.prev_state != self.state or self.last_pwm != int((self.command_left_rear_pwm + self.command_right_rear_pwm) /2.0):
            msg = MotorsOrderAdas()
            msg.command_right_rear_pwm = self.command_right_rear_pwm
            msg.command_left_rear_pwm = self.command_left_rear_pwm

            msg.state = self.state
            msg.active = self.active

            self.publisher_motors_order.publish(msg)

        self.prev_state = self.state
        self.last_pwm = int((self.command_left_rear_pwm + self.command_right_rear_pwm) /2.0)
        return


def main(args=None):
    rclpy.init(args=args)

    speed_regulation = SpeedRegulation()

    rclpy.spin(speed_regulation)

    speed_regulation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()