import rclpy
from rclpy.node import Node
from interfaces.msg import DetectedLane, MotorsOrder, MotorsOrderAdas
from rcl_interfaces.msg import SetParametersResult
# Image to debug
HEIGHT = 480
WIDTH = 640
from sensor_msgs.msg import Image

TIMER_REFRESH = 0.5

STATE_NOTHING = "state_nothing"
STATE_LANE_CROSSED_LEFT = "state_lane_crossed_left"
STATE_LANE_CROSSED_RIGHT = "state_lane_crossed_right"

MINIMUM_LANE_CROSS_DISTANCE_PX = 24
HOLD_TIME = 3

class lane_crossing_assist(Node):
    def __init__(self):
        #Initialization of the node
        super().__init__('lane_crossing_assist_node')

        # Publishing and subscribing
        self.publisher_motors_order = self.create_publisher(MotorsOrderAdas, 'motors_order_lca', 10)
        self.subscription = self.create_subscription(DetectedLane,'detected_lanes_position', self.lane_position_callback, 10)
        
        # Class Variable 
        self.right_lane_valid = False
        self.left_lane_valid = False
        
        self.lane_crossed_right = False
        self.lane_crossed_left = False

        self.left_lane = 2

        self.left_zone = [241, 1000]
        self.right_zone = [-1000, 240]

        self.lane1_point_ref = 0
        self.lane2_point_ref = 0

        self.left_lane_point_ref = 0
        self.right_lane_point_ref = 0

        self.state = STATE_NOTHING
        self.prev_state = STATE_NOTHING

        self.prev_steering = 0
        self.steering_hold = 0

        self.no_lanes = True

        # Add environnement variables
        self.declare_parameter("right_lane_crossing_threshold", WIDTH*0.4)
        self.declare_parameter("left_lane_crossing_threshold", WIDTH*0.6)
        self.declare_parameter("zone_width", 400)
        self.declare_parameter("minimum_lane_cross_distance_px", MINIMUM_LANE_CROSS_DISTANCE_PX)
        self.declare_parameter("timer_refresh_period", TIMER_REFRESH)
        self.declare_parameter("hold_time", HOLD_TIME)
        self.declare_parameter("image_height", HEIGHT)
        self.declare_parameter("image_width", WIDTH)
        self.add_on_set_parameters_callback(self.param_callback)

        self.right_lane_crossing_threshold = self.get_parameter("right_lane_crossing_threshold").value
        self.left_lane_crossing_threshold = self.get_parameter("left_lane_crossing_threshold").value
        self.zone_width = self.get_parameter("zone_width").value
        self.minimum_lane_cross_distance_px = self.get_parameter("minimum_lane_cross_distance_px").value
        self.timer_refresh_period = self.get_parameter("timer_refresh_period").value
        self.hold_time = self.get_parameter("hold_time").value
        self.image_height = self.get_parameter("image_height").value
        self.image_width = self.get_parameter("image_width").value

        # Timer
        self.timer = self.create_timer(self.timer_refresh_period, self.reset_zones)
        
        self.get_logger().info("lane_crossing_assist_node READY")

    def reset_zones(self):

        self.get_logger().info("0.5s without new lanes, resetting zones")

        self.left_zone = [241, 1000]
        self.right_zone = [-1000, 240]
        self.steering_hold = 1

        if not(self.no_lanes) and self.prev_steering != 0:
            self.publish_adas(STATE_NOTHING, 0, False)

        self.no_lanes = True

    # ------------------Environment Variable -----------
    def param_callback(self, params):
        for p in params:
            if p.name == "right_lane_crossing_threshold":
                self.right_lane_crossing_threshold = p.value
            elif p.name == "left_lane_crossing_threshold":
                self.left_lane_crossing_threshold = p.value
            elif p.name == "zone_width":
                self.zone_width = p.value
            elif p.name == "minimum_lane_cross_distance_px":
                self.minimum_lane_cross_distance_px = p.value
            elif p.name == "timer_refresh_period":
                self.get_logger().info("Cannot change timer period after run")
            elif p.name == "hold_time":
                self.hold_time = p.value
            elif p.name == "image_height":
                self.image_height = p.value
            elif p.name == "image_width":
                self.image_width = p.value
        self.get_logger().info(f"Changed parameter {p.name} to {p.value}")
        return SetParametersResult(successful=True)

    def lane_position_callback(self,lanes : DetectedLane):

        _, _, self.lane1_point_ref = self.compute_lane_point_ref(lanes.lane1_bottom_x, lanes.lane1_bottom_y, lanes.lane1_top_x, lanes.lane1_top_y, lanes.lane1_valid)
        _, _, self.lane2_point_ref = self.compute_lane_point_ref(lanes.lane2_bottom_x, lanes.lane2_bottom_y, lanes.lane2_top_x, lanes.lane2_top_y, lanes.lane2_valid)
        
        lanes.lane1_valid &= (self.lane1_point_ref != None) and (-1000 < self.lane1_point_ref < 1000)
        lanes.lane2_valid &= (self.lane2_point_ref != None) and (-1000 < self.lane2_point_ref < 1000)

        if (lanes.lane1_valid or lanes.lane2_valid):

            self.no_lanes = False

            self.timer.reset()
        
            self.determine_side_lane(lanes)
            self.lane_crossing_test()

            steering_angle, command_steering = self.calculate_steering_angle()
            self.publish_adas(self.state, steering_angle, command_steering)

        elif self.state != STATE_NOTHING :

            self.state = STATE_NOTHING
            self.publish_adas(STATE_NOTHING, 0, False)

    def determine_side_lane(self, lanes):
        # if both lanes are present, then update the zones
        if lanes.lane1_valid and lanes.lane2_valid:

            self.left_lane = 2

            self.left_lane_point_ref = self.lane2_point_ref
            self.right_lane_point_ref = self.lane1_point_ref
            
            self.left_lane_valid = True
            self.right_lane_valid = True

            self.left_zone[0] = self.left_lane_point_ref - self.zone_width
            self.left_zone[1] = self.left_lane_point_ref + self.zone_width

            self.right_zone[0] = self.right_lane_point_ref - self.zone_width
            self.right_zone[1] = self.right_lane_point_ref + self.zone_width

        # if lane 1 is valid, is it left or right ? Default : right
        if lanes.lane1_valid and not(lanes.lane2_valid):
            if self.left_zone[0] >= self.lane1_point_ref >= self.left_zone[1]:
                self.left_lane_point_ref = self.lane1_point_ref
                self.left_lane_valid = True
                self.right_lane_valid = False
            else:
                self.right_lane_point_ref = self.lane1_point_ref
                self.right_lane_valid = True
                self.left_lane_valid = False

        # if lane 2 is valid, is it left or right ? Default : left
        if lanes.lane2_valid and not(lanes.lane1_valid):
            if self.right_zone[0] >= self.lane2_point_ref >= self.right_zone[1]:
                self.right_lane_point_ref = self.lane2_point_ref
                self.right_lane_valid = True
                self.left_lane_valid = False
            else:
                self.left_lane_point_ref = self.lane2_point_ref 
                self.left_lane_valid = True
                self.right_lane_valid = False
                

    def compute_lane_point_ref(self, x1, y1, x2, y2, valid):
        if (y2 - y1) == 0 :
            return None, None, None
        a = (x2 - x1) / (y2 - y1)
        b = x1 - a * y1
        if a == 0 :
            return None, None, None
        point_ref = a * self.image_height + b
        self.get_logger().info(f"a = {a:.2f}, b = {b:.2f}, y1 = {y1:.2f}, y2 = {y2:.2f}, x1 = {x1:.2f}, x2 = {x2:.2f}, point_ref = {point_ref:.2f}")
        return a, b, point_ref
    
    def lane_crossing_test(self):

        self.prev_state = self.state

        if (self.right_lane_valid and self.right_lane_point_ref > self.right_lane_crossing_threshold):
            self.state = STATE_LANE_CROSSED_RIGHT
            self.get_logger().info(f"/////////RIGHT lane crossing detected | right_lane_point_ref = {self.right_lane_point_ref:.2f} > {self.right_lane_crossing_threshold}")
        else:
            self.state = STATE_NOTHING

        if (self.left_lane_valid and self.left_lane_point_ref < self.left_lane_crossing_threshold):
            self.state = STATE_LANE_CROSSED_LEFT
            self.get_logger().info(f"/////////LEFT lane crossing detected | left_lane_point_ref = {self.left_lane_point_ref:.2f} < {self.left_lane_crossing_threshold}")
        elif self.state != STATE_LANE_CROSSED_RIGHT:
            self.lane_crossed_left = False
            self.state = STATE_NOTHING

    def publish_adas(self, state, command_steering_angle, command_steering):

            command_steering_angle = self.prev_steering
            
            if self.steering_hold >= 0:
                command_steering = True
                self.steering_hold -= 1

            if self.steering_hold == 0:
                command_steering_angle = 0

            if (self.prev_state == self.state and self.state == STATE_NOTHING and self.steering_hold == -1):
                return None
            
            msg = MotorsOrderAdas()

            self.get_logger().info(f"PUBLISH / Steering angle = {command_steering_angle} / Steering hold = {self.steering_hold}")

            msg.command_steering_angle = command_steering_angle
            msg.command_steering = command_steering
            msg.state = state
            msg.active = True

            self.publisher_motors_order.publish(msg)

    def calculate_steering_angle(self):

        steering_angle = 0
        # self.get_logger().info(f"Calculating steering angle state = {self.state}")

        if self.state == STATE_LANE_CROSSED_RIGHT:
            deviation_px = self.right_lane_point_ref - self.right_lane_crossing_threshold
            self.get_logger().info(f"Calculating steering angle w/ deviation {deviation_px}")
            if abs(deviation_px) > self.minimum_lane_cross_distance_px:
                steering_angle = 0.73 * deviation_px - 17.64
                steering_angle = min(max(int(steering_angle), -127), 127)
                # self.get_logger().info(f"Steering angle = {steering_angle}\n")
                self.prev_steering = steering_angle
                self.steering_hold = self.hold_time
                return int(steering_angle), True

        if self.state == STATE_LANE_CROSSED_LEFT:
            deviation_px = self.left_lane_point_ref - self.left_lane_crossing_threshold
            self.get_logger().info(f"Calculating steering angle w/ deviation {deviation_px}")
            if abs(deviation_px) > self.minimum_lane_cross_distance_px:
                steering_angle = 0.73 * deviation_px + 17.64
                steering_angle = min(max(int(steering_angle), -127), 127)
                # self.get_logger().info(f"Steering angle = {steering_angle}\n")
                self.prev_steering = steering_angle
                self.steering_hold = self.hold_time
                return steering_angle, True

        return steering_angle, False
            

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