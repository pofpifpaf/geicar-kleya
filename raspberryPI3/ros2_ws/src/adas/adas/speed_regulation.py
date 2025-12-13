import rclpy
from rclpy.node import Node
import math

from interfaces.msg import MotorsOrder, JoystickOrder, MotorsFeedback, MotorsOrderAdas 

#Global Variable
SPEED_TOLERANCE = 0.05 # Tolérance vitesse 5%
SPEED_CORRECTION = 0.5 #a voir si c'est ok
STATE_NOTHING = "state_nothing"
STATE_MODE_OFF = "state_mode_off"
STATE_NOT_OK = "state_not_ok"
STATE_OK = "state_ok"

REFRESH_PERIOD = 0.002  #valeur comme collision ?

class speed_regulation(Node):
    def __init__(self):
        #Initialization of the node
        super().__init__('speed_regulation_node')
        #Create a topic for the command of regulation
        self.publisher_motors_order = self.create_publisher(MotorsOrderAdas, 'motors_order_speed', 10)

        #Subscription to the node regulation need
        self.subscription = self.create_subscription(MotorsOrder,'motors_order_raw', self.motors_order_callback, 10)
        self.subscription = self.create_subscription(JoystickOrder,'joystick_order', self.joystick_order_callback,10)
        self.subscription = self.create_subscription(MotorsFeedback,'motors_feedback', self.motors_feedback_callback,10)
        self.subscription  # prevent unused variable warning

        #init variable joystick
        self.start = False
        self.mode = 0           # 0 -> Manual ; 1 -> Autonomous ; 2 -> Steering Calibration
        self.throttle = 0.0
        self.reverse = False

        #régulation vitesse
        self.target_speed = 0.0
        self.auto_speed = False
        self.prev_auto_speed = False

        #commande moteur
        self.motor_right_rear_pwm = 0
        self.motor_left_rear_pwm = 0
        self.motor_steering_angle = 0
        self.left_rear_speed = 0
        self.right_rear_speed = 0


        self.motor_right_rear_pwm_offset = 0
        self.motor_left_rear_pwm_offset = 0
        self.steering_angle_offset = 0

        self.max_pwm = 50 #max_pwm relatif

        self.emergency_stop = False
        self.active = False
        self.state = STATE_NOTHING
        self.prev_state = STATE_NOTHING

        self.timer = self.create_timer(REFRESH_PERIOD, self.regulate_speed)

        self.get_logger().info("speed_regulation_node READY")

    ####################################
    ######### Callback Section #########
    ####################################


    def motors_order_callback(self, motors_order : MotorsOrder):
        self.motor_right_rear_pwm = motors_order.right_rear_pwm
        self.motor_left_rear_pwm = motors_order.left_rear_pwm
        self.motor_steering_angle = motors_order.steering_angle

    def joystick_order_callback(self, joystick_order: JoystickOrder):
        self.start = joystick_order.start
        self.mode = joystick_order.mode
        self.throttle = joystick_order.throttle
        self.reverse = joystick_order.reverse

        #Manuel a auto
        self.prev_auto_speed = self.auto_speed
        self.auto_speed = (self.mode == 1)   # mode == 1 -> mode auto

        if self.auto_speed and not self.prev_auto_speed:
            # current_speed = desired speed
            current_speed = (self.left_rear_speed + self.right_rear_speed) / 2.0
            self.target_speed = current_speed
            self.get_logger().info(f"Automatic mode ON: speed = {self.target_speed:.2f}")

        #Auto a manuel
        if not self.auto_speed and self.prev_auto_speed:
            self.get_logger().info("Automatic mode OFF")


    def motors_feedback_callback(self, msg: MotorsFeedback):
        self.left_rear_speed = msg.left_rear_speed
        self.right_rear_speed = msg.right_rear_speed
            

    
    ####################################
    ######## Speed Implementation ########
    ####################################
      

    def regulate_speed(self):
        """Régulation avec tolérance ±5 %
        si pas en mode auto on ne fait rien
        si tolerance ok fait rien
        si trop haut/bas on applique un gain de correction
        """
        self.state = STATE_NOTHING
        self.active = False

            #Vérifie la tolérance
        actual_speed = (self.left_rear_speed + self.right_rear_speed) / 2.0
        delta = self.target_speed - actual_speed
        tolerance = SPEED_TOLERANCE * abs(self.target_speed)

        if not self.auto_speed:
            self.state = STATE_MODE_OFF
            if self.state != self.prev_state:
                self.get_logger().info("Automatic mode OFF")
            return
        else:
            #Vitesse ok
            if abs(delta) < tolerance:
                self.state = STATE_OK
                if self.state != self.prev_state:
                    self.get_logger().info("Tolerane respecter")

            #Vitesse trop haute/basse : correction
            else:
                pwm = SPEED_CORRECTION * delta 

                self.motor_right_rear_pwm_offset = int(pwm)
                self.motor_left_rear_pwm_offset = int(pwm)
                self.active = True

                self.state = STATE_NOT_OK
                if self.state != self.prev_state:
                    self.get_logger().info("Tolerance dépassée")

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

    speed_regulation_node = speed_regulation()

    rclpy.spin(speed_regulation_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    speed_regulation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    