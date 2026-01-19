import rclpy
from rclpy.node import Node

from interfaces.msg import DetectedLane

LANE1_BOTTOM_X = "lane1_bottom_x"
LANE1_BOTTOM_Y = "lane1_bottom_y"

LANE1_TOP_X = "lane1_top_x"
LANE1_TOP_Y = "lane1_top_y"

LANE1_VALID = "lane1_valid"

LANE2_BOTTOM_X = "lane2_bottom_x"
LANE2_BOTTOM_Y = "lane2_bottom_y"

LANE2_TOP_X = "lane2_top_x"
LANE2_TOP_Y = "lane2_top_y"

LANE2_VALID = "lane2_valid"


class lane_filtering (Node):

    def __init__(self):

	    # Initialization of the node
	    super().__init__('lane_filtering_node')
	    self.lane_pub = self.create_publisher(DetectedLane, 'detected_lanes_position', 10)
	    self.subscription = self.create_subscription(DetectedLane,'detected_lanes_position_raw', self.detected_lanes_position_raw_callback, 10)
	    self.subscription  # prevent unused variable warning

	    # Var init
	    self.lane = {}
	    self.lane_filtered = {}
	    self.lane_prev = {}

	    self.confident = True

	# --------------------------------------------------------------
	# Callback for detected_lanes_position_raw topic
	# --------------------------------------------------------------
    def detected_lanes_position_raw_callback(self, msg : DetectedLane):

    	self.lane[LANE1_BOTTOM_X] = msg.lane1_bottom_x
		self.lane[LANE1_BOTTOM_Y] = msg.lane1_bottom_y

		self.lane[LANE1_TOP_X] = msg.lane1_top_x
		self.lane[LANE1_TOP_Y] = msg.lane1_top_y

		self.lane[LANE1_VALID] = msg.lane1_valid

		self.lane[LANE2_BOTTOM_X] = msg.lane2_bottom_x
		self.lane[LANE2_BOTTOM_Y] = msg.lane2_bottom_y

		self.lane[LANE2_TOP_X] = msg.lane2_top_x
		self.lane[LANE2_TOP_Y] = msg.lane2_top_y

		self.lane[LANE2_VALID] = msg.lane2_valid

		filter_lane_position_input()
	
	# --------------------------------------------------------------
	# Publish self.lane_filtered into detected_lanes_position topic
	# --------------------------------------------------------------
	def publish_lane_message(self):

		filtered_lane_message = DetectedLane()

		filtered_lane_message.lane1_bottom_x = self.lane_filtered[LANE1_BOTTOM_X]
		filtered_lane_message.lane1_bottom_y = self.lane_filtered[LANE1_BOTTOM_Y]

		filtered_lane_message.lane1_top_x = self.lane_filtered[LANE1_TOP_X]
		filtered_lane_message.lane1_top_y = self.lane_filtered[LANE1_TOP_Y]

		filtered_lane_message.lane1_valid = self.lane_filtered[LANE1_VALID]

		filtered_lane_message.lane2_bottom_x = self.lane_filtered[LANE2_BOTTOM_X]
		filtered_lane_message.lane2_bottom_y = self.lane_filtered[LANE2_BOTTOM_Y]

		filtered_lane_message.lane2_top_x = self.lane_filtered[LANE2_TOP_X]
		filtered_lane_message.lane2_top_y = self.lane_filtered[LANE2_TOP_Y]

		filtered_lane_message.lane2_valid = self.lane_filtered[LANE2_VALID]		

		self.lane_pub.publish(filtered_lane_message)


	# --------------------------------------------------------------
	# Lane output filter
	# --------------------------------------------------------------
	def filter_lane_position_input(self):

		# test for angle < 30 degrees

		# minimum lane length

		# if you detect two lanes, they should:
			# be roughly parallel
			# have reasonable distance between them
				# abs(lane1_angle - lane2_angle) < threshold
				# abs(lane1_bottom_x - lane2_bottom_x) within [min_width, max_width]

		# outlier rejection
			# abs(x_new - x_filtered) < threshold

		# exponential moving averaging 
			# x_filt = alpha * x_new + (1 - alpha) * x_prev (alpha = 0.4)

		# publish only when confident
		if self.confident:
			publish_lane_message()





def main(args=None):
    rclpy.init(args=args)

    lane_filtering_node = lane_filtering()

    rclpy.spin(lane_filtering_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lane_filtering_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()