import rclpy
from rclpy.node import Node
from interfaces.msg import DetectedLane
from rcl_interfaces.msg import SetParametersResult
# Image to debug
from sensor_msgs.msg import Image


class  lane_crossing_detection(Node):
    def __init__(self):
        #Initialization of the node
        super().__init__('lane_crossing_detection_node')
        #self.subscription  # prevent unused variable warning
        self.subscription = self.create_subscription(DetectedLane,'detected_lanes_position', self.lane_position_callback, 10)
        # Subscription to image for debug 
        self.image_pub = self.create_publisher(Image,'image_lane_detection',10)

        
        # Add environnement variables
        self.declare_parameter("left_lane_crossing_threshold", 384)
        self.declare_parameter("right_lane_crossing_threshold", 896)
        self.add_on_set_parameters_callback(self.param_callback)

        self.left_lane_crossing = self.get_parameter("left_lane_crossing_threshold").value
        self.right_lane_crossing = self.get_parameter("right_lane_crossing_threshold").value
        
        self.get_logger().info("lane_crossing_detection_node READY")

    # ------------------Environment Variable -----------
    def param_callback(self, params):
        for p in params:
            if p.name == "left_lane_crossing_threshold":
                self.left_lane_crossing_threshold = p.value
            elif p.name == "right_lane_crossing_threshold":
                self.right_lane_crossing_threshold = p.value

        return SetParametersResult(successful=True)

    def lane_position_callback(self,lanes : DetectedLane):
        # Get left lane coordinate
        self.left_lane_bottom_x = DetectedLane.lane1_bottom_x
        self.left_lane_bottom_y = DetectedLane.lane1_bottom_y
        self.left_lane_top_x = DetectedLane.lane1_top_x
        self.left_lane_top_y = DetectedLane.lane1_top_y
        self.left_lane_valid = DetectedLane.lane1_valid
        # Get right lane coordinate
        self.right_lane_bottom_x = DetectedLane.lane2_bottom_x
        self.right_lane_bottom_y = DetectedLane.lane2_bottom_y
        self.right_lane_top_x = DetectedLane.lane2_top_x
        self.right_lane_top_y = DetectedLane.lane2_top_y
        self.right_lane_valid = DetectedLane.lane2_valid

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