# TO DO : La commande et verifier les machines à états
import rclpy
from rclpy.node import Node
from interfaces.msg import MotorsOrder, ECompass 
from sensor_msgs.msg import Imu

#Global Variable
# Variables from the car characteristics
T_SAMPLE = 0.1 #s ou 100ms
MIN_DEVIATION_Z_ANG_VEL = 0.75  #rad/s
MAX_STEER = 127                    # steer range
INTERMEDIATE_TIMEOUT = 2           # seconds
MIN_ESP_ACTIVE_TIME = 0.5          # seconds before disabling
# ESP parameters
SPEED_PERCENTAGE = 0.5
HEADING_TOLERANCE = 1.0            # tolerance for the heading
SIZE_BUFFER_HEADING = 20
REF_HEADING_RATE = 9.0                    # heading difference to activate ESP
DELAY_INDEX = 10                   # wait for 1 sec stability before deactivating ESP

class Esp(Node):
    def __init__(self):
        #Initialization of the node
        super().__init__('esp_node')
        #Create a topic for the control/command of the ESP
        self.publisher_motors_order = self.create_publisher(MotorsOrder, 'motors_order', 10)

        #Subscription to the node ESP need
        self.subscription = self.create_subscription(MotorsOrder,'motors_order_raw', self.motors_order_callback, 10)
        self.subscription = self.create_subscription(Imu,'/imu/data', self.imu_callback,10)
        self.subscription = self.create_subscription(ECompass,'imu/ecompass', self.ecompass_callback,10)
        self.subscription  # prevent unused variable warning

        #init variable des commandes moteurs
        self.motor_right_rear_pwm = 50
        self.motor_left_rear_pwm = 50
        self.motor_steering_angle = 0

        # Heading variables to detect sudden deviation
        self.last_heading = None
        self.heading = None
        # Reference heading when ESP is activated
        self.reference_heading = None
        # Buffer for heading stability check
        self.heading_buffer = []
        self.stable_count = 0

        # ESP state variables
        self.esp_intermediate_state = False
        self.esp_active = False
        self.intermediate_start_time = None

        self.get_logger().info("esp_node READY")

    # ------------------ CALLBACKS ------------------
    def motors_order_callback(self, motors_order : MotorsOrder):

        self.motor_right_rear_pwm = motors_order.right_rear_pwm
        self.motor_left_rear_pwm = motors_order.left_rear_pwm
        self.motor_steering_angle = motors_order.steering_angle


    def imu_callback(self, msg):

        if self.esp_intermediate_state :
            # Check for timeout before confirming deviation
            if not self.intermediate_timeout_ok():
                return  # exit if timeout reached
            self.deviation_confirmation(msg)

    def ecompass_callback(self, ecompass: ECompass):
        current = ecompass.heading

        if self.last_heading is not None:
            self.detect_heading_jump(self.last_heading, current)

        # Now update last heading
        self.last_heading = current

        # Update buffer & check deactivation
        self.update_heading_buffer(current)
        self.check_esp_deactivation()


    # ------- HEADING BUFFER FOR DEACTIVATION CONDITION -------

    def update_heading_buffer(self, heading):
        self.heading_buffer.append(heading)
        if len(self.heading_buffer) > SIZE_BUFFER_HEADING:
            self.heading_buffer.pop(0)

    def is_heading_stable(self):
        if len(self.heading_buffer) < SIZE_BUFFER_HEADING:
            return False
        
        avg = sum(self.heading_buffer) / len(self.heading_buffer)
        return abs(avg - self.reference_heading) < HEADING_TOLERANCE
    

    # ------------------ ESP Implementation ------------------ 

    def sign(self, num):
        return -1 if num < 0 else 1

    def detect_heading_jump(self, prev, current):
        heading_rate = (current - prev) / T_SAMPLE  # deg/s
        # Check for sudden heading change
        if abs(heading_rate) > REF_HEADING_RATE and not self.esp_active and not self.esp_intermediate_state :
            self.reference_heading = prev # set reference to previous stable heading 
            self.esp_intermediate_state = True
            self.intermediate_start_time = self.get_clock().now()   # START TIMER
            self.get_logger().info("ESP IN INTERMEDIATE STATE — heading drift detected")

    def intermediate_timeout_ok(self):
        if not self.esp_intermediate_state:
            return False  # Just in case

        now = self.get_clock().now()
        elapsed = (now - self.intermediate_start_time).nanoseconds * 1e-9

        if elapsed > INTERMEDIATE_TIMEOUT:
            self.esp_intermediate_state = False
            self.get_logger().info("ESP INTERMEDIATE TIMEOUT — no deviation confirmed")
            return False

        return True


    def check_esp_deactivation(self):
        if not self.esp_active:
            return
        
        # Check heading stability using buffer average
        if abs(self.last_heading - self.reference_heading) < HEADING_TOLERANCE :
            self.stable_count += 1
        else:
            self.stable_count = 0  # reset if unstable
        
        if self.stable_count >= DELAY_INDEX:  # stable for required samples
            self.esp_active = False
            self.reference_heading = self.last_heading # update reference
            self.get_logger().info("ESP DEACTIVATED — heading stable")



    def deviation_confirmation(self, msg):
        # deviation detection
        rotation_rate = msg.angular_velocity.z
        if abs(rotation_rate) > MIN_DEVIATION_Z_ANG_VEL and not self.esp_active :
                self.esp_active = True
                self.esp_intermediate_state = False
                self.get_logger().info(
                    f"ESP ACTIVATED | ref_heading = {self.reference_heading:.2f}° | angular_velocity_z = {rotation_rate:.2f} rad/s"
                )

    def trajectory_control(self):
        # #Complete change in the command with pallier
        pass
        # # Update motors order
        # # Value for Motors Control
        # msg = MotorsOrder()
        # msg.right_rear_pwm = self.motor_right_rear_pwm
        # msg.left_rear_pwm = self.motor_left_rear_pwm
        # msg.steering_angle = 0 # Angle [-128;127]

        # # Publish output
        # self.publisher_motors_order.publish(msg)
        
        # # Message in the logger only if deviation
        # self.get_logger().info(
        #     f"ESP Activated | rate={self.deviation_rate:.1f} deg/s"
        # )
        # self.get_logger().info(f"New Motors Order: {msg}")


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
    
