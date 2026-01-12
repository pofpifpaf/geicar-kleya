import rclpy
from rclpy.node import Node
import math

from interfaces.msg import MotorsOrder, JoystickOrder, MotorsFeedback, MotorsOrderAdas
from std_srvs.srv import SetBool  #Service pour ON/OFF 

# Global Variable
SPEED_TOLERANCE = 0.05  #Tolérance vitesse 5%
STOP = 50

# States
STATE_NOTHING = "state_nothing"
STATE_MANUAL_MODE = "state_manual_mode"
STATE_AUTOMATIC_MODE_CORRECTING = "state_automatic_mode_correcting"
STATE_AUTOMATIC_MODE_NO_CORRECTING = "state_automatic_mode_no_correcting"

REFRESH_PERIOD = 0.01  #10 ms
MIN_TOLERANCE = 1.0  #Minimum tolerance in RPM


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
        self.target_PWM = 0
        self.auto_mode_on = False  

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

        self.last_pwm_offset = (self.motor_left_rear_pwm_offset + self.motor_right_rear_pwm_offset) /2.0

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
            self.auto_mode_on = True
            self.target_speed = (self.left_rear_speed + self.right_rear_speed) / 2.0
            self.target_PWM = (self.motor_left_rear_pwm + self.motor_right_rear_pwm) / 2.0
            self.get_logger().info(f"ACC ON : target speed={self.target_speed:.2f} and target PWM={self.target_PWM:.2f})")
        else:
            self.auto_mode_on = False
            self.motor_right_rear_pwm_offset = 0
            self.motor_left_rear_pwm_offset = 0
            #self.get_logger().info("ACC OFF")

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


        #MODE OFF (MANUEL)
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
        
        #MODE ON (AUTO)

        ##Calcul de la vitesse actuelle
        actual_speed = (self.left_rear_speed + self.right_rear_speed) / 2.0
        
        ##Calcul de l'erreur et de la tolérance
        delta = self.target_speed - actual_speed
        tolerance = max(MIN_TOLERANCE, SPEED_TOLERANCE * abs(self.target_speed))
        

        ## Vitesse dans les marges de tolerance => Pas de correction
        if abs(delta) <= tolerance:
            self.state = STATE_AUTOMATIC_MODE_NO_CORRECTING
            if self.state != self.prev_state:
                self.get_logger().info("Speed within tolerance")

        ## Vitesse hors des marges de tolerance
        else:
            user_PWM = (self.motor_left_rear_pwm + self.motor_right_rear_pwm) / 2.0
            pwm_offset = self.target_PWM - user_PWM

            ## Si l'utilisateur essaye d'aller en arrière ou de dépasser la vitesse cible => Pas de correction
            if pwm_offset < 0 or user_PWM < STOP:
                self.state = STATE_AUTOMATIC_MODE_NO_CORRECTING
                if self.state != self.prev_state:
                    self.get_logger().info("ACC ON: user controlling beyond desired speed")
            
            ## Sinon on corrige la vitesse
            else:
                self.state = STATE_AUTOMATIC_MODE_CORRECTING
                self.active = True
                
                self.motor_right_rear_pwm_offset = int(pwm_offset)
                self.motor_left_rear_pwm_offset = int(pwm_offset)

                if self.state != self.prev_state:
                    self.get_logger().info("Correcting speed")

        #Publishing
        if (self.prev_state != self.state) or self.last_pwm_offset != (self.motor_left_rear_pwm_offset + self.motor_right_rear_pwm_offset) /2.0:
            msg = MotorsOrderAdas()
            msg.offset_right_rear_pwm = self.motor_right_rear_pwm_offset
            msg.offset_left_rear_pwm = self.motor_left_rear_pwm_offset
            msg.offset_steering_angle = self.steering_angle_offset

            msg.max_pwm = self.max_pwm
            msg.emergency_stop = self.emergency_stop
            msg.active = self.active

            self.publisher_motors_order.publish(msg)

        self.prev_state = self.state
        self.last_pwm_offset = (self.motor_left_rear_pwm_offset + self.motor_right_rear_pwm_offset) /2.0
        return


def main(args=None):
    rclpy.init(args=args)

    speed_regulation = speed_regulation()

    rclpy.spin(speed_regulation)

    speed_regulation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()