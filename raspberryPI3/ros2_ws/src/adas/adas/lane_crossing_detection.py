import rclpy
from rclpy.node import Node
from interfaces.msg import DetectedLane, MotorsOrder
from rcl_interfaces.msg import SetParametersResult
# Image to debug
from sensor_msgs.msg import Image


class  lane_crossing_detection(Node):
    def __init__(self):
        #Initialization of the node
        super().__init__('lane_crossing_detection_node')
        #self.subscription  # prevent unused variable warning
        self.subscription = self.create_subscription(DetectedLane,'detected_lanes_position', self.lane_position_callback, 10)
        # Subscribe Joystick command for extra security
        self.sub_raw = self.create_subscription(MotorsOrder, 'motors_order_raw', self.motors_order_raw_callback, 10)
        # Subscription to image for debug 
        self.image_pub = self.create_publisher(Image,'image_lane_detection',10)
        # Callback for the trajectory command
        self.control_timer = self.create_timer(0.1, self.crossing_test_callback)
        # Init variable state
        self.active_start_time = None

        self.left_lane_bottom_x = 0.0
        self.left_lane_top_x = 0.0
        self.left_lane_valid = False
        self.right_lane_bottom_x = 0.0
        self.right_lane_top_x = 0.0
        self.right_lane_valid = False
        self.raw_steering_angle = 0.0
        # Class Variable
        self.height = 480
        self.width = 640
        self.lane_crossed_left = False
        self.lane_crossed_right = False
        self.security_counter = 0
        # Add environnement variables
        self.declare_parameter("left_lane_crossing_threshold", self.width*0.1)
        self.declare_parameter("right_lane_crossing_threshold", self.width*0.9)
        self.add_on_set_parameters_callback(self.param_callback)

        self.left_lane_crossing_threshold = self.get_parameter("left_lane_crossing_threshold").value
        self.right_lane_crossing_threshold = self.get_parameter("right_lane_crossing_threshold").value
        
        self.get_logger().info("lane_crossing_detection_node READY")

    # ------------------Environment Variable -----------
    def crossing_test_callback(self):
        self.lane_crossing_test()
        self.check_active_timeout()

    def check_active_timeout(self):
        if self.active_start_time is None:
            return
    def param_callback(self, params):
        for p in params:
            if p.name == "left_lane_crossing_threshold":
                self.left_lane_crossing_threshold = p.value
            elif p.name == "right_lane_crossing_threshold":
                self.right_lane_crossing_threshold = p.value

        return SetParametersResult(successful=True)

    def lane_position_callback(self,lanes : DetectedLane):
        # Get left lane coordinate
        self.left_lane_bottom_x = lanes.lane1_bottom_x
        self.left_lane_top_x = lanes.lane1_top_x
        self.left_lane_valid = lanes.lane1_valid
        
        # Get right lane coordinate
        self.right_lane_bottom_x = lanes.lane2_bottom_x
        self.right_lane_top_x = lanes.lane2_top_x
        self.right_lane_valid = lanes.lane2_valid
    
    def motors_order_raw_callback(self, msg):
        self.raw_steering_angle = msg.steering_angle
           
    def lane_crossing_test(self):
        if self.left_lane_bottom_x > self.left_lane_crossing_threshold:
             self.lane_crossed_left = True
        if self.right_lane_bottom_x < self.right_lane_crossing_threshold:
            self.lane_crossed_right = True
        # Condition where no line is detected and the joystick go right
        # Malfunction ??
        # TO DO: ADD message
        # self.get_logger().info("Test")
            
            

def main(args=None):
    rclpy.init(args=args)

    lane_crossing_detection_node = lane_crossing_detection()

    rclpy.spin(lane_crossing_detection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lane_crossing_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()