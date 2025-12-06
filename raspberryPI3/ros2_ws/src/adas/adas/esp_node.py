import rclpy
from rclpy.node import Node
import math

from interfaces.msg import MotorsOrder, JoystickOrder, ECompass 
from sensor_msgs.msg import Imu          #IMU

#Global Variable
MIN_DETECTION_DEVIATION = 0.7  #rad/s
 

class Esp(Node):
    def __init__(self):
        #Initialization of the node
        super().__init__('esp_node')
        #Create a topic for the control/command of the ESP
        self.publisher_motors_order = self.create_publisher(MotorsOrder, 'motors_order_ESP', 10)

        #Subscription to the node ESP need
        self.subscription = self.create_subscription(MotorsOrder,'motors_order_raw', self.motors_order_callback, 10)
        self.subscription = self.create_subscription(Imu,'/imu/data', self.imu_callback,10)
        self.subscription = self.create_subscription(JoystickOrder,'joystick_order', self.joystick_order_callback,10)
        self.subscription = self.create_subscription(ECompass,'imu/ecompass', self.ecompass_callback,10)
        self.subscription  # prevent unused variable warning

        #init variable des commandes moteurs
        self.motor_right_rear_pwm = 50
        self.motor_left_rear_pwm = 50
        self.motor_steering_angle = 0
        #init variable for command
        self.command_error = 0
        """TO DO : Init the desired_cap ?"""

        self.get_logger().info("esp_node READY")

    ####################################
    ######### Callback Section #########
    ####################################
    def motors_order_callback(self, motors_order : MotorsOrder):

        self.motor_right_rear_pwm = motors_order.right_rear_pwm
        self.motor_left_rear_pwm = motors_order.left_rear_pwm
        self.motor_steering_angle = motors_order.steering_angle

    def joystick_order_callback(self, joystick_order : JoystickOrder):
        self.steer = joystick_order.steer
        self.reverse = joystick_order.reverse

    def imu_callback(self, msg):
        angular_velocity = msg.angular_velocity.z
    
    def ecompass_callback(self,ecompass : ECompass):
        #TO DO: Get the value of desired_cap
        #TO DO: Convertir la valeur en relatif
        pass
    
    ####################################
    ######## ESP Implementation ########
    ####################################

    def detect_deviation(self, msg: Imu): 
        #utilisez l'angular velocity plus grand que 0.7
        angular_velocity = msg.angular_velocity.z
        if abs(angular_velocity) >= MIN_DETECTION_DEVIATION:
            angular_velocity_deg = math.degrees(angular_velocity) #je switch rad en degré
            self.get_logger().info(f"Deviation : {angular_velocity_deg:.2f} °/s detected")
            #self.trajectory_control(command_err) TO DO
            return True
        return False



    # For now a simple feedback no control law
    def trajectory_control(self):       #Arguments to be added : command_error
        # Command to be defined
        """if pas de probleme d'angle :
                msg = MotorsOrder()
                msg.right_rear_pwm = self.motor_right_rear_pwm
                msg.left_rear_pwm = self.motor_left_rear_pwm
                msg.steering_angle = self.motor_steering_angle
                self.publisher_motors_order.publish(msg)
                return
            else 
                angle actuel - angle de reference
                faire +/- 180"""
        self.get_logger().info("ESP Trajectory Control Activated")


def main(args=None):
    rclpy.init(args=args)

    esp_node = Esp()

    rclpy.spin(esp_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    esp_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    