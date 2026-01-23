import rclpy
from rclpy.node import Node
from interfaces.msg import DetectedLane, MotorsOrder
from rcl_interfaces.msg import SetParametersResult
# Image to debug
HEIGHT = 480
WIDTH = 640
from sensor_msgs.msg import Image


class lane_crossing_assist(Node):
    def __init__(self):
        #Initialization of the node
        super().__init__('lane_crossing_assist_node')
        #self.subscription  # prevent unused variable warning
        self.subscription = self.create_subscription(DetectedLane,'detected_lanes_position', self.lane_position_callback, 10)
        # Init variable for DetectedLane messages
        self.left_lane_valid = False
        self.right_lane_valid = False
        # Class Variable 
        self.lane_crossed_left = False
        self.lane_crossed_right = False

        self.right_lane = 2

        self.right_zone = [241, 1000]
        self.left_zone = [-1000, 240]

        self.lane1_point_ref = 0
        self.lane2_point_ref = 0

        self.right_lane_point_ref = 0
        self.left_lane_point_ref = 0

        # Add environnement variables
        self.declare_parameter("left_lane_crossing_threshold", WIDTH*0.4)
        self.declare_parameter("right_lane_crossing_threshold", WIDTH*0.6)
        self.add_on_set_parameters_callback(self.param_callback)

        self.left_lane_crossing_threshold = self.get_parameter("left_lane_crossing_threshold").value
        self.right_lane_crossing_threshold = self.get_parameter("right_lane_crossing_threshold").value

        self.timer = self.create_timer(0.5, self.reset_zones)
        
        self.get_logger().info("lane_crossing_assist_node READY")

    def reset_zones(self):
        self.get_logger().info("0.5s without new lanes, resetting zones")
        self.right_zone = [241, 1000]
        self.left_zone = [-1000, 240]

    # ------------------Environment Variable -----------
    def param_callback(self, params):
        for p in params:
            if p.name == "left_lane_crossing_threshold":
                self.left_lane_crossing_threshold = p.value
            elif p.name == "right_lane_crossing_threshold":
                self.right_lane_crossing_threshold = p.value

        return SetParametersResult(successful=True)

    def lane_position_callback(self,lanes : DetectedLane):

        # self.get_logger().info("Receiving new lanes")

        _, _, self.lane1_point_ref = self.compute_lane_point_ref(lanes.lane1_bottom_x, lanes.lane1_bottom_y, lanes.lane1_top_x, lanes.lane1_top_y, lanes.lane1_valid)
        _, _, self.lane2_point_ref = self.compute_lane_point_ref(lanes.lane2_bottom_x, lanes.lane2_bottom_y, lanes.lane2_top_x, lanes.lane2_top_y, lanes.lane2_valid)
        
        lanes.lane1_valid &= (self.lane1_point_ref != None)
        lanes.lane2_valid &= (self.lane2_point_ref != None)

        if (lanes.lane1_valid or lanes.lane2_valid):
            # self.get_logger().info("And one of them is valid")
            self.timer.reset()
        
            self.determine_side_lane(lanes)
            self.lane_crossing_test()

    def determine_side_lane(self, lanes):
        # if both lanes are present, then update the zones
        if lanes.lane1_valid and lanes.lane2_valid:
            # self.get_logger().info(f"Updating zones")

            self.right_lane = 2

            self.right_lane_point_ref = self.lane2_point_ref
            self.left_lane_point_ref = self.lane1_point_ref
            
            self.right_lane_valid = True
            self.left_lane_valid = True

            self.right_zone[0] = self.right_lane_point_ref - 400
            self.right_zone[1] = self.right_lane_point_ref + 400

            self.left_zone[0] = self.left_lane_point_ref - 400
            self.left_zone[1] = self.left_lane_point_ref + 400

        # if lane 1 is valid, is it right or left ? Default : left
        if lanes.lane1_valid and not(lanes.lane2_valid):
            # self.get_logger().info("ONE LANE ### lane1")
            if self.right_zone[0] >= self.lane1_point_ref >= self.right_zone[1]:
                # self.get_logger().info(f"INSIDE ZONE #### Selecting right lane from point_ref {self.right_lane_point_ref:.2f} px and zone {self.right_zone[1] - self.right_zone[0]:.2f}")
                self.right_lane_point_ref = self.lane1_point_ref
                self.right_lane_valid = True
                self.left_lane_valid = False
            else:
                self.left_lane_point_ref = self.lane1_point_ref
                self.left_lane_valid = True
                self.right_lane_valid = False

        # if lane 2 is valid, is it right or left ? Default : right
        if lanes.lane2_valid and not(lanes.lane1_valid):
            # self.get_logger().info("ONE LANE ### lane2")
            if self.left_zone[0] >= self.lane2_point_ref >= self.left_zone[1]:
                # self.get_logger().info(f"INSIDE ZONE #### Selecting left lane from point_ref {self.left_lane_point_ref:.2f} px and zone {self.left_zone[1] - self.left_zone[0]:.2f}")
                self.left_lane_point_ref = self.lane2_point_ref
                self.left_lane_valid = True
                self.right_lane_valid = False
            else:
                self.right_lane_point_ref = self.lane2_point_ref 
                self.right_lane_valid = True
                self.left_lane_valid = False
                
        # self.get_logger().info(f"left_zone = [{self.left_zone[0]:.2f}, {self.left_zone[1]:.2f}] || right_zone = [{self.right_zone[0]:.2f}, {self.right_zone[1]:.2f}]")

    def compute_lane_point_ref(self, x1, y1, x2, y2, valid):
        if (y2 - y1) == 0 :
            return None, None, None
        a = (x2 - x1) / (y2 - y1)
        b = x1 - a * y1
        if a == 0 :
            return None, None, None
        point_ref = a * HEIGHT + b
        self.get_logger().info(f"a = {a:.2f}, b = {b:.2f}, y1 = {y1:.2f}, y2 = {y2:.2f}, x1 = {x1:.2f}, x2 = {x2:.2f}, point_ref = {point_ref:.2f}")
        return a, b, point_ref
    
    def lane_crossing_test(self):
        if (self.left_lane_valid and self.left_lane_point_ref > self.left_lane_crossing_threshold):
            self.get_logger().info(f"/////////LEFT lane crossing detected | left_lane_point_ref = {self.left_lane_point_ref:.2f} > {self.left_lane_crossing_threshold}")
        else:
            self.lane_crossed_left = False
        if (self.right_lane_valid and self.right_lane_point_ref < self.right_lane_crossing_threshold):
            self.get_logger().info(f"/////////RIGHT lane crossing detected | right_lane_point_ref = {self.right_lane_point_ref:.2f} < {self.right_lane_crossing_threshold}")
            self.lane_crossed_right = True
        else:
            self.lane_crossed_right = False

            
            

def main(args=None):
    rclpy.init(args=args)

    lane_crossing_assist_node = lane_crossing_assist()

    rclpy.spin(lane_crossing_assist_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lane_crossing_assist_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()