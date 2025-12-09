import rclpy
from rclpy.node import Node
import math

from interfaces.msg import MotorsOrder, JoystickOrder, ECompass 
from sensor_msgs.msg import Imu          #IMU

#Global Variable
T_SAMPLE = 0.1 #s ou 100ms
MIN_DETECTION_DEVIATION = 1  #rad/s
SPEED_PERCENTAGE = 0.5
HEADING_TOLERANCE = 3.0            # tolerance for the heading
MAX_STEER = 127                    # steer range
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
        self.esp_active = False
        self.reference_heading = None
        self.manual_steer = 0.0 # user input
        self.get_logger().info("esp_node READY")

    # ------------------ CALLBACKS ------------------
    def motors_order_callback(self, motors_order : MotorsOrder):

        self.motor_right_rear_pwm = motors_order.right_rear_pwm
        self.motor_left_rear_pwm = motors_order.left_rear_pwm
        self.motor_steering_angle = motors_order.steering_angle

    def joystick_order_callback(self, joystick_order : JoystickOrder):
        self.manual_steer = joystick_order.steer

    def imu_callback(self, msg):
        rotation_rate = msg.angular_velocity.z
        
        # deviation detection
        if abs(rotation_rate) > MIN_DETECTION_DEVIATION:
            if not self.esp_active and self.last_heading is not None:
                self.esp_active = True
                self.reference_heading = self.last_heading
                self.get_logger().info(
                    f"ESP ACTIVATED | ref_heading = {self.reference_heading:.2f}°"
                )


    def ecompass_callback(self,ecompass : ECompass):
        #TO DO: Get the value of desired_cap
        heading = ecompass.heading
        if self.last_heading == None :
            self.last_heading = heading
        else:
            self.deviation_rate = (self.last_heading-heading)/T_SAMPLE    #heading rate calculation
            self.last_heading = heading         #update for next iteration
            # Run ESP only when active
            if self.esp_active:
                self.trajectory_control()

                # Deactivation condition
                if abs(self.reference_heading - heading) < HEADING_TOLERANCE:
                    self.esp_active = False
                    self.get_logger().info("ESP DEACTIVATED — heading corrected")


    # ------------------ ESP Implementation ------------------ 

    def detect_deviation(self, msg: Imu): 
        #utilisez l'angular velocity plus grand que 0.5
        angular_velocity = msg.angular_velocity.z
        if abs(angular_velocity) >= MIN_DETECTION_DEVIATION:
            return True
        return False

    def sign(self, num):
        return -1 if num < 0 else 1

    def trajectory_control(self):
        #Complete change in the command

        # Update motors order
        # Value for Motors Control
        msg = MotorsOrder()
        msg.right_rear_pwm = self.motor_right_rear_pwm
        msg.left_rear_pwm = self.motor_left_rear_pwm
        msg.steering_angle = 0 # Angle [-128;127]

        # Publish output
        self.publisher_motors_order.publish(msg)
        
        # Message in the logger only if deviation
        self.get_logger().info(
            f"ESP Activated | rate={self.deviation_rate:.1f} deg/s"
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
    
