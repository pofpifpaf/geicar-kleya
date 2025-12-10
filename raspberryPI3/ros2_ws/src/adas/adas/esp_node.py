# TO DO : Buffer pour le heading
# TO DO : La commande
# TO DO : Double detection avec le heading en premier critère puis timer pour voir si angular velocity
#               Pour distinguer le tournant du choc
# T_SAMPLE changer avec les vrais trames reçu
# TO DO : Ajouter un timer pour sortir de l'état intermédiaire si pas de déviation après un certain temps
import rclpy
from rclpy.node import Node
from interfaces.msg import MotorsOrder, JoystickOrder, ECompass 
from sensor_msgs.msg import Imu

#Global Variable
T_SAMPLE = 0.1 #s ou 100ms
MIN_DETECTION_DEVIATION = 1  #rad/s
SPEED_PERCENTAGE = 0.5
HEADING_TOLERANCE = 3.0            # tolerance for the heading
MAX_STEER = 127                    # steer range
SIZE_BUFFER_HEADING = 20
REF_HEADING_DIFF = 5.0                    # heading difference to activate ESP
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
        self.heading = None
        self.reference_heading = None
        self.heading_buffer = []
        
        # ESP state variables
        self.esp_intermediate_state = False
        self.esp_active = False
        

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

        if self.esp_intermediate_state :
            self.deviation_confirmation(msg)

    def ecompass_callback(self, ecompass: ECompass):
        self.heading = ecompass.heading     
        # Initialization of last heading
        if self.last_heading is None:
            self.last_heading = self.heading
            return
        # Compare heading variation
        self.compare_heading_variation(self.last_heading, self.heading)
        # Mise à jour pour la prochaine itération
        self.last_heading = self.heading



    # ------------------ ESP Implementation ------------------ 

    def sign(self, num):
        return -1 if num < 0 else 1

    def compare_heading_variation(self, prev, current):
        if abs(prev - current) > REF_HEADING_DIFF:
            self.esp_intermediate_state = True
            self.get_logger().info("ESP IN INTERMEDIATE STATE — heading change detected")

        elif self.esp_active:
            # ToDO: Compare the headings buffer to see if the heading is stable
            self.esp_intermediate_state = False
            self.esp_active = False
            self.get_logger().info("ESP DEACTIVATED — heading stable")
            # Deactivation condition
            # if abs(self.reference_heading - current) < HEADING_TOLERANCE:
            #     self.esp_active = False
            #     self.get_logger().info("ESP DEACTIVATED — heading corrected")

    def deviation_confirmation(self, msg):
        # deviation detection
        rotation_rate = msg.angular_velocity.z
        if abs(rotation_rate) > MIN_DETECTION_DEVIATION:
            if not self.esp_active and self.last_heading is not None:
                self.esp_active = True
                self.reference_heading = self.last_heading
                self.get_logger().info(
                    f"ESP ACTIVATED | ref_heading = {self.reference_heading:.2f}°"
                )

    def trajectory_control(self):
        #Complete change in the command with pallier

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
    
