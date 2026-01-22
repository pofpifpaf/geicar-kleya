import rclpy
from rclpy.node import Node
from interfaces.msg import DetectedLane, MotorsOrder
from rcl_interfaces.msg import SetParametersResult
# Image to debug
HEIGHT = 480
WIDTH = 640
from sensor_msgs.msg import Image


class  lane_crossing_detection(Node):
    def __init__(self):
        #Initialization of the node
        super().__init__('lane_crossing_detection_node')
        #self.subscription  # prevent unused variable warning
        self.subscription = self.create_subscription(DetectedLane,'detected_lanes_position', self.lane_position_callback, 10)
        # Subscription to image for debug 
        self.image_pub = self.create_publisher(Image,'image_lane_detection',10)
        # Init variable for DetectedLane messages
        self.left_lane_valid = False
        self.right_lane_valid = False
        # Class Variable 
        self.lane_crossed_left = False
        self.lane_crossed_right = False
        # Add environnement variables
        self.declare_parameter("left_lane_crossing_threshold", WIDTH*0.4)
        self.declare_parameter("right_lane_crossing_threshold", WIDTH*0.6)
        self.add_on_set_parameters_callback(self.param_callback)

        self.left_lane_crossing_threshold = self.get_parameter("left_lane_crossing_threshold").value
        self.right_lane_crossing_threshold = self.get_parameter("right_lane_crossing_threshold").value
        
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
        # Get lane validity
        self.left_lane_valid = lanes.lane1_valid
        self.right_lane_valid = lanes.lane2_valid
        # Get the projection or x coordinate at the bottom of the frame for lane crossing
        if self.left_lane_valid or self.right_lane_valid:
            self.calculation_ref_points(lanes)
            self.lane_crossing_test()

    def calculation_ref_points(self,lanes):
        # Convert to coefficient
        left_a, left_b = self.compute_lane_coefficients(lanes.lane1_bottom_x,lanes.lane1_bottom_y,lanes.lane1_top_x,lanes.lane1_top_y,lanes.lane1_valid)
        right_a, right_b = self.compute_lane_coefficients(lanes.lane2_bottom_x,lanes.lane2_bottom_y,lanes.lane2_top_x,lanes.lane2_top_y,lanes.lane2_valid)
        if (left_a is not None and left_b is not None) :
            self.left_point_ref = (HEIGHT + left_b) / left_a
        if (right_b is not None and right_b != None):
            self.right_point_ref = (HEIGHT + right_b) / right_a

    # Problème ici parce que le collège c'était y'a longtemps
    def compute_lane_coefficients(self, x1, y1, x2, y2, valid):
        if not valid :
            return None, None
        a = (y2 - y1) / (x2 - x1) 
        b = x1 - a * y1
        return a, b
    
    def lane_crossing_test(self):
        if (self.left_lane_valid and self.left_point_ref > self.left_lane_crossing_threshold):
            self.get_logger().info(
                f"Left lane crossing detected | left_point_ref = {self.left_point_ref:.2f} px"
            )
        else:
            self.lane_crossed_left = False
        if (self.right_lane_valid and self.right_point_ref < self.right_lane_crossing_threshold):
            self.get_logger().info(
                f"Right lane crossing detected | right_point_ref = {self.right_point_ref:.2f} px"
            )
            self.lane_crossed_right = True
        else:
            self.lane_crossed_right = False

            
            

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