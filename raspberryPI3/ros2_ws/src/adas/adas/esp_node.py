"""
ESP Node for Vehicle Stability Control usging State machine for gei-car
with IDLE, INTERMEDIATE, and ACTIVE states. 

Topics:
- Subscribed: 
    - 'motors_order_raw' (MotorsOrder)
    - '/imu/data' (Imu)
    - 'imu/ecompass' (ECompass)
- Published:
    - 'motors_order_esp' (MotorsOrderAdas)

Author: Caroline Nguyen
Date: 13/12/2025
"""

import rclpy
from rclpy.node import Node
from interfaces.msg import MotorsOrder, ECompass , MotorsOrderAdas
from sensor_msgs.msg import Imu
from rcl_interfaces.msg import SetParametersResult

#Global Variable from the car characteristics
T_SAMPLE = 0.1 #s ou 100ms
MAX_STEER = 127                    # steer range

# Define ESP states
ESP_STATE_IDLE = "state_idle"
ESP_STATE_INTERMEDIATE = "state_intermediate"
ESP_STATE_ACTIVE = "state_active"

class Esp(Node):
    def __init__(self):
        #Initialization of the node
        super().__init__('esp_node')
        #Create a topic for the control/command of the ESP
        self.publisher_motors_order = self.create_publisher(MotorsOrderAdas, 'motors_order_esp', 10)
        #Subscription to the node ESP need
        self.subscription = self.create_subscription(MotorsOrder,'motors_order_raw', self.motors_order_callback, 10)
        self.subscription = self.create_subscription(Imu,'/imu/data', self.imu_callback,10)
        self.subscription = self.create_subscription(ECompass,'imu/ecompass', self.ecompass_callback,10)
        self.subscription  # prevent unused variable warning

        # Callback for the trajectory command
        self.control_timer = self.create_timer(
            T_SAMPLE,
            self.control_loop
        )

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
        self.state = ESP_STATE_IDLE
        self.intermediate_start_time = None
        
        self.direction = None


        # Add environnement variables
        self.declare_parameter("heading_tolerance", 2.0)
        self.declare_parameter("ref_heading_rate", 9.0)
        self.declare_parameter("size_buffer_heading", 20)
        self.declare_parameter("stabilization_index", 4)
        self.declare_parameter("intermediate_timout", 2)
        self.declare_parameter("min_deviation_z_ang_vel", 0.6) # Value to confirm to deviation
        self.declare_parameter("active_timeout", 20)
        self.add_on_set_parameters_callback(self.param_callback)

        self.heading_tolerance = self.get_parameter("heading_tolerance").value
        self.ref_heading_rate = self.get_parameter("ref_heading_rate").value
        self.size_buffer_heading = self.get_parameter("size_buffer_heading").value
        self.stabilization_index = self.get_parameter("stabilization_index").value
        self.intermediate_timout = self.get_parameter("intermediate_timout").value
        self.min_deviation_z_ang_vel = self.get_parameter("min_deviation_z_ang_vel").value
        self.active_timeout = self.get_parameter("active_timeout").value
        
        self.get_logger().info("esp_node READY")

    # ------------------Environment Variable -----------
    def param_callback(self, params):
        for p in params:
            if p.name == "heading_tolerance":
                self.heading_tolerance = p.value
            elif p.name == "size_buffer_heading":
                self.size_buffer_heading = p.value
            elif p.name == "ref_heading_rate":
                self.ref_heading_rate = p.value
            elif p.name == "stabilization_index":
                self.stabilization_index = p.value
            elif p.name == "intermediate_timout":
                self.intermediate_timout = p.value
            elif p.name == "min_deviation_z_ang_vel":
                self.min_deviation_z_ang_vel = p.value
            elif p.name == "active_timeout":
                self.active_timeout = p.value

        return SetParametersResult(successful=True)
 
    # ------------------ CALLBACKS ------------------
    def motors_order_callback(self, motors_order : MotorsOrder):

        self.motor_right_rear_pwm = motors_order.right_rear_pwm
        self.motor_left_rear_pwm = motors_order.left_rear_pwm
        self.motor_steering_angle = motors_order.steering_angle


    def imu_callback(self, msg):

        if self.state == ESP_STATE_INTERMEDIATE :
            # Check for timeout before confirming deviation
            if not self.intermediate_timout_ok():
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

    def control_loop(self):
        if self.state == ESP_STATE_ACTIVE:
            self.trajectory_control()
            self.check_active_timeout()


    # ------- HEADING BUFFER FOR DEACTIVATION CONDITION -------

    def update_heading_buffer(self, heading):
        self.heading_buffer.append(heading)
        if len(self.heading_buffer) > self.size_buffer_heading:
            self.heading_buffer.pop(0)

    def is_heading_stable(self):
        if len(self.heading_buffer) < self.size_buffer_heading:
            return False
        
        avg = sum(self.heading_buffer) / len(self.heading_buffer)
        return abs(avg - self.reference_heading) <self.heading_tolerance
    

    # ------------------ ESP Implementation ------------------ 

    def sign(self, num):
        return -1 if num < 0 else 1

    def detect_heading_jump(self, prev, current):
        heading_rate = (current - prev) / T_SAMPLE  # deg/s
        # Check for sudden heading change
        if abs(heading_rate) > self.ref_heading_rate and self.state == ESP_STATE_IDLE :
            self.reference_heading = prev # set reference to previous stable heading 
            self.state = ESP_STATE_INTERMEDIATE
            self.intermediate_start_time = self.get_clock().now()   # START TIMER
            self.get_logger().info("ESP IN INTERMEDIATE STATE — heading drift detected")

    def intermediate_timout_ok(self):
        if self.state != ESP_STATE_INTERMEDIATE:
            return False  # Just in case

        now = self.get_clock().now()
        elapsed = (now - self.intermediate_start_time).nanoseconds * 1e-9

        if elapsed > self.intermediate_timout:
            self.state = ESP_STATE_IDLE
            self.get_logger().info("ESP INTERMEDIATE TIMEOUT — no deviation confirmed")
            return False

        return True
    
    def check_active_timeout(self):
        if self.active_start_time is None:
            return

        now = self.get_clock().now()
        elapsed = (now - self.active_start_time).nanoseconds * 1e-9

        if elapsed > self.active_timeout:
            self.state = ESP_STATE_IDLE
            self.active_start_time = None
            self.stable_count = 0
            self.reference_heading = self.last_heading
            self.get_logger().info("ESP ACTIVE TIMEOUT — deactivating ESP")
            self.send_msg()  # reset motors


    def check_esp_deactivation(self):
        if self.state != ESP_STATE_ACTIVE:
            return
        
        # Check heading stability using buffer average
        if abs(self.reference_heading - self.last_heading ) < self.heading_tolerance :
            self.stable_count += 1
        else:
            self.stable_count = 0  # reset if unstable
        
        if self.stable_count >= self.stabilization_index:  # stable for required samples
            self.state = ESP_STATE_IDLE
            
            #msg state = false
            self.send_msg ()
            self.stable_count = 0
            self.reference_heading = self.last_heading # update reference
            self.get_logger().info("ESP DEACTIVATED — heading stable")



    def deviation_confirmation(self, msg):
        # deviation detection
        rotation_rate = msg.angular_velocity.z
        if abs(rotation_rate) > self.min_deviation_z_ang_vel and self.state == ESP_STATE_INTERMEDIATE :
                self.state = ESP_STATE_ACTIVE
                self.direction = self.sign(rotation_rate)
                self.active_start_time = self.get_clock().now()  # start active timer
                self.get_logger().info(
                    f"ESP ACTIVATED | ref_heading = {self.reference_heading:.2f}° | angular_velocity_z = {rotation_rate:.2f} rad/s"
                )

    def trajectory_control(self):
        # #Complete change in the command with pallier
        
        # 1) Heading error
        error = self.reference_heading - self.last_heading   # degrees
        # 2) Steering correction (proportional)
        if abs(error) > 30:
            steer_correction = int(self.direction * MAX_STEER)
        elif abs(error) > 15:
            steer_correction = int(self.direction * MAX_STEER / 1.5)
        else:
            steer_correction = int(self.direction * MAX_STEER / 2)
        # log for the trajectory command
        self.get_logger().info(
            f"ESP CTRL | error={error:.2f}° | steer={steer_correction}"
        )
        self.send_msg(steer = steer_correction,state=self.state, active=True)
    
    def send_msg (self, steer=0, state=ESP_STATE_IDLE, active=False) :
        # # Update motors order
        # # Value for Motors Control
        msg = MotorsOrderAdas()
        msg.command_steering_angle = steer
        msg.state = state
        msg.command_steering = active
        msg.active = active
        # # Publish output
        self.publisher_motors_order.publish(msg)
        
        # Message in the logger only if deviation
        #self.get_logger().info(f"New Motors Order sent: {msg}")


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
    
