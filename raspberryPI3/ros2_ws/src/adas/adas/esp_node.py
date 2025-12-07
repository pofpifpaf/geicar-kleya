import rclpy
from rclpy.node import Node
import math

from interfaces.msg import MotorsOrder, JoystickOrder, ECompass 
from sensor_msgs.msg import Imu          #IMU

#Global Variable
T_SAMPLE = 0.1 #s ou 100ms
MIN_DETECTION_DEVIATION = 0.5  #rad/s
SPEED_PERCENTAGE = 0.5

class Esp(Node):
    def __init__(self):
        #Initialization of the node
        super().__init__('esp_node')
        #Create a topic for the control/command of the ESP
        self.publisher_motors_order = self.create_publisher(MotorsOrder, 'motors_order', 10)

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
        self.deviation_rate = None
        self.last_heading = None
        self.command_error = 0
        self.deviation_rate = 0
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
         # Commented this out to test the trajectory control only
         if (self.detect_deviation(msg) == True): 
            self.trajectory_control()

    def ecompass_callback(self,ecompass : ECompass):
        #TO DO: Get the value of desired_cap
        if self.last_heading == None :
            self.last_heading = ecompass.heading
        else:
            heading = ecompass.heading
            self.deviation_rate = (self.last_heading-heading)/T_SAMPLE    #heading rate calculation
            self.last_heading = heading         #update for next iteration

        #self.trajectory_control()
    
    ####################################
    ######## ESP Implementation ########
    ####################################

    def detect_deviation(self, msg: Imu): 
        #utilisez l'angular velocity plus grand que 0.5
        angular_velocity = msg.angular_velocity.z
        if abs(angular_velocity) >= MIN_DETECTION_DEVIATION:
            angular_velocity_deg = math.degrees(angular_velocity) #je switch rad en degré
            self.get_logger().info(f"Deviation : {angular_velocity_deg:.2f} °/s detected")
            return True
        return False

    def sign(self, num):
        return -1 if num < 0 else 1

    def trajectory_control(self):
        # Security for initialisation
        if self.deviation_rate is None:
            return False
        # Value for Motors Control
        msg = MotorsOrder()
        msg.right_rear_pwm = self.motor_right_rear_pwm
        msg.left_rear_pwm = self.motor_left_rear_pwm
        msg.steering_angle = self.motor_steering_angle

        # Intermediate variables for the motors order modification
        rate = abs(self.deviation_rate) # Used as a reference for deviation severity
        direction = self.sign(self.deviation_rate)

        # Drifting like in movies yeeeeeeeeeh
        if rate > 60:
            steer = 1 * direction
        # Driving on ice
        elif rate > 0.8:
            steer = 0.4 * direction
        # Driving like a Marseillais
        elif rate > 20:
            steer = 0.6 * direction
        # No deviation
        else:
            return False #ESP was not activated, joystick order only

        # Update motors order
        #msg.right_rear_pwm = 
        #msg.left_rear_pwm  = int(msg.left_rear_pwm *SPEED_PERCENTAGE)
        msg.steering_angle = int(steer*127) # Angle [-128;127]

        # Publish output
        self.publisher_motors_order.publish(msg)
        # Message in the logger only if deviation
        self.get_logger().info(
            f"ESP Activated | rate={self.deviation_rate:.1f} deg/s | steer={steer:.2f}"
        )
        self.get_logger().info(f"New Motors Order: {msg}")


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
    