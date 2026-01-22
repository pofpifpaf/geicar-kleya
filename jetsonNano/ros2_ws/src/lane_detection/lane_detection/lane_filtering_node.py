import rclpy
from rclpy.node import Node

from interfaces.msg import DetectedLane

import numpy as np

# if image_output:
import cv2
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

image_output = True

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

		# pub
		self.lane_pub = self.create_publisher(DetectedLane, 'detected_lanes_position', 10)
		if image_output: 
			self.image_pub = self.create_publisher(Image,'image_lane_detection_filtered',10)

		# sub
		self.subscription = self.create_subscription(DetectedLane,'detected_lanes_position_raw', self.detected_lanes_position_raw_callback, 10)
		if image_output:
			self.subscription = self.create_subscription(CompressedImage,'image_raw/compressed', self.image_raw_callback, 10)
		self.subscription  # prevent unused variable warning

		# param
		self.declare_parameter("angle_threshold", 30)
		self.declare_parameter("length_threshold", 30)
		self.declare_parameter("min_lane_distance", 0)
		self.declare_parameter("ema_alpha", 0.4)
		self.add_on_set_parameters_callback(self.param_callback)

		self.bridge = CvBridge()

		# var
		self.angle_threshold = self.get_parameter("angle_threshold").value
		self.angle_threshold = self.get_parameter("length_threshold").value
		self.min_lane_distance = self.get_parameter("min_lane_distance").value
		self.ema_alpha = self.get_parameter("ema_alpha").value


		self.lane = {LANE1_BOTTOM_X: 0,
                         LANE1_BOTTOM_Y: 0,
                         LANE1_TOP_X: 0,
                         LANE1_TOP_Y: 0,
                         LANE1_VALID: False,
                         LANE2_BOTTOM_X: 0,
                         LANE2_BOTTOM_Y: 0,
                         LANE2_TOP_X: 0,
                         LANE2_TOP_Y: 0,
                         LANE2_VALID: False,
                         }
		self.lane_filtered = {LANE1_BOTTOM_X: 0,
                         LANE1_BOTTOM_Y: 0,
                         LANE1_TOP_X: 0,
                         LANE1_TOP_Y: 0,
                         LANE1_VALID: False,
                         LANE2_BOTTOM_X: 0,
                         LANE2_BOTTOM_Y: 0,
                         LANE2_TOP_X: 0,
                         LANE2_TOP_Y: 0,
                         LANE2_VALID: False,
                         }
		self.lane_prev = {LANE1_BOTTOM_X: 0,
                         LANE1_BOTTOM_Y: 0,
                         LANE1_TOP_X: 0,
                         LANE1_TOP_Y: 0,
                         LANE1_VALID: False,
                         LANE2_BOTTOM_X: 0,
                         LANE2_BOTTOM_Y: 0,
                         LANE2_TOP_X: 0,
                         LANE2_TOP_Y: 0,
                         LANE2_VALID: False,
                         }

		self.confident_lane1 = True
		self.confident_lane2 = True

		self.confident = True

	def param_callback(self, params):
		for p in params:
			if p.name == "angle_threshold":
				self.angle_threshold = p.value
				self.get_logger().info(f"Parameter angle_threshold changed to {self.angle_threshold}")
			if p.name == "length_threshold":
				self.length_threshold = p.value
				self.get_logger().info(f"Parameter angle_threshold changed to {self.length_threshold}")
		return SetParametersResult(successful=True)

	def image_raw_callback(self, image : CompressedImage):
		
		np_arr = np.frombuffer(image.data, np.uint8)
		self.inputimage = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV BGR
		self.header = image.header



	def draw_lines(self, image, thickness=8, color=(0, 0, 255), y_min_ratio=0.55):

		line_image = np.zeros_like(image)

		if self.lane[LANE1_VALID]:
			cv2.line(line_image, (self.lane[LANE1_BOTTOM_X], self.lane[LANE1_BOTTOM_Y]), (self.lane[LANE1_TOP_X], self.lane[LANE1_TOP_Y]), color, thickness)
		if self.lane[LANE2_VALID]:
			cv2.line(line_image, (self.lane[LANE2_BOTTOM_X], self.lane[LANE2_BOTTOM_Y]), (self.lane[LANE2_TOP_X], self.lane[LANE2_TOP_Y]), color, thickness)

		if not(self.lane[LANE1_VALID]) and not(self.lane[LANE2_VALID]):
			return image

		return cv2.addWeighted(image, 0.8, line_image, 1.0, 0.0)

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

		self.filter_lane_position_input()

		msg = self.bridge.cv2_to_imgmsg(self.draw_lines(self.inputimage), encoding='bgr8')
		msg.header = self.header
		self.image_pub.publish(msg)
	
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
	# EMA Filter on a specific key
	# --------------------------------------------------------------
	def ema(self, key):
		# x_filt = alpha * x_new + (1 - alpha) * x_prev (alpha = 0.4)
		self.lane_filtered[key] = self.ema_alpha * self.lane[key] + (1 - self.ema_alpha) * self.lane_prev[key]


	# --------------------------------------------------------------
	# Lane output filter
	# --------------------------------------------------------------
	def filter_lane_position_input(self):

		self.confident_lane1 = self.lane[LANE1_VALID]
		self.confident_lane2 = self.lane[LANE2_VALID]

		self.confident = self.confident_lane1 and self.confident_lane2

		# test for angle < 30 degrees (by default)
		if (self.lane[LANE1_TOP_Y] - self.lane[LANE1_BOTTOM_Y]) != 0:
			slope_lane1 = (self.lane[LANE1_TOP_X] - self.lane[LANE1_BOTTOM_X])/(self.lane[LANE1_TOP_Y] - self.lane[LANE1_BOTTOM_Y])
		else :
			slope_lane1 = 0.0
		if (self.lane[LANE2_TOP_Y] - self.lane[LANE2_BOTTOM_Y]) != 0:
			slope_lane2 = (self.lane[LANE2_TOP_X] - self.lane[LANE2_BOTTOM_X])/(self.lane[LANE2_TOP_Y] - self.lane[LANE2_BOTTOM_Y])
		else :
			slope_lane2 = 0.0

		# # angle in rad -> converted to deg
		# angle = np.arctan(abs((slope_lane1 - slope_lane2)/(1 + slope_lane1 * slope_lane2))) * (180 / np.pi)
		# if slope_lane1 != 0 :
		# 	angle = ((np.pi/2 - np.arctan(1/slope_lane1)) * (180 / np.pi))
		# 	self.confident_lane1 &= (angle > 25 and angle < 90) or (angle < -25 and angle > -90)
		# 	if not(self.confident_lane1):
		# 		self.get_logger().info(f"Lane invalidated")
		# 	self.get_logger().info(f"Angle lane1 {angle}")

		# if slope_lane2 != 0 :
		# 	angle = ((np.pi/2 - np.arctan(1/slope_lane2)) * (180 / np.pi))
		# 	self.confident_lane2 &= (angle > 25 and angle < 90) or (angle < -25 and angle > -90)
		# 	if not(self.confident_lane2):
		# 		self.get_logger().info(f"Lane invalidated")
		# 	self.get_logger().info(f"Angle lane2 {angle}")

		# angle in rad -> converted to deg
		
		if slope_lane1 != 0 :
			angle1 = abs(np.arctan2(self.lane[LANE1_TOP_Y] - self.lane[LANE1_BOTTOM_Y], self.lane[LANE1_TOP_X] - self.lane[LANE1_BOTTOM_X]) * (180 / np.pi))
			self.confident_lane1 &= (angle1 > 45 and angle1 < 130)
			if not(self.confident_lane1):
				self.get_logger().info(f"Lane invalidated")
			self.get_logger().info(f"Angle lane1 {angle1} \n dx = {self.lane[LANE1_TOP_X]} - {self.lane[LANE1_BOTTOM_X]} dy = {self.lane[LANE1_TOP_Y]} - {self.lane[LANE1_BOTTOM_Y]}")
		if slope_lane2 != 0 :
			angle2 = abs(np.arctan2(self.lane[LANE2_TOP_Y] - self.lane[LANE2_BOTTOM_Y], self.lane[LANE2_TOP_X] - self.lane[LANE2_BOTTOM_X]) * (280 / np.pi))
			self.confident_lane2 &= (angle2 > 45 and angle2 < 130)
			if not(self.confident_lane2):
				self.get_logger().info(f"Lane invalidated")
			self.get_logger().info(f"Angle lane2 {angle2} \n dx = {self.lane[LANE2_TOP_X]} - {self.lane[LANE2_BOTTOM_X]} dy = {self.lane[LANE2_TOP_Y]} - {self.lane[LANE2_BOTTOM_Y]}")

		# self.confident_lane1 = self.lane[LANE1_VALID]
		# self.confident_lane2 = self.lane[LANE2_VALID]


		# self.confident &= angle > self.angle_threshold

		# minimum lane length
		# length_lane1 = sqrt((self.lane[LANE1_TOP_X] - self.lane[LANE1_BOTTOM_X])**2 + (self.lane[LANE1_TOP_Y] - self.lane[LANE1_BOTTOM_Y])**2)
		# length_lane2 = sqrt((self.lane[LANE2_TOP_X] - self.lane[LANE2_BOTTOM_X])**2 + (self.lane[LANE2_TOP_Y] - self.lane[LANE2_BOTTOM_Y])**2)

		# self.confident_lane1 &= length_lane1 > self.length_threshold
		# self.confident_lane2 &= length_lane2 > self.length_threshold

		# if you detect two lanes, they should:
			# be roughly parallel
			# have reasonable distance between them
				# abs(lane1_angle - lane2_angle) < threshold
				# abs(lane1_bottom_x - lane2_bottom_x) within [min_width, max_width]
		# distance_lanes = abs(self.lane[LANE1_BOTTOM_X] - self.lane[LANE2_BOTTOM_X])
		# self.confident &= (distance_lanes > self.min_lane_distance) and (distance_lanes < self.min_lane_distance)

		# exponential moving averaging 
			# x_filt = alpha * x_new + (1 - alpha) * x_prev (alpha = 0.4)
		# if self.confident_lane1:

		# 	self.ema(LANE1_TOP_X)
		# 	self.ema(LANE1_TOP_Y)
		# 	self.ema(LANE1_BOTTOM_X)
		# 	self.ema(LANE1_BOTTOM_Y)

		# if self.confident_lane2:

		# 	self.ema(LANE2_TOP_X)
		# 	self.ema(LANE2_TOP_Y)
		# 	self.ema(LANE2_BOTTOM_X)
		# 	self.ema(LANE2_BOTTOM_Y)
			
		# outlier rejection
			# abs(x_new - x_filtered) < threshold
			# TODO

		self.lane[LANE1_VALID] = self.confident_lane1
		self.lane[LANE2_VALID] = self.confident_lane2

		# publish only when confident
		if self.confident:
			self.publish_lane_message()

		if self.lane[LANE1_VALID]:

			self.lane_prev[LANE1_BOTTOM_X] = self.lane[LANE1_BOTTOM_X]
			self.lane_prev[LANE1_TOP_X] = self.lane[LANE1_TOP_X]

			self.lane_prev[LANE1_BOTTOM_Y] = self.lane[LANE1_BOTTOM_Y]
			self.lane_prev[LANE1_TOP_Y] = self.lane[LANE1_TOP_Y]

			self.lane_prev[LANE1_VALID] = True

		if self.lane[LANE2_VALID]:

			self.lane_prev[LANE2_BOTTOM_X] = self.lane[LANE2_BOTTOM_X]
			self.lane_prev[LANE2_TOP_X] = self.lane[LANE2_TOP_X]

			self.lane_prev[LANE2_BOTTOM_Y] = self.lane[LANE2_BOTTOM_Y]
			self.lane_prev[LANE2_TOP_Y] = self.lane[LANE2_TOP_Y]

			self.lane_prev[LANE2_VALID] = True

	# --------------------------------------------------------------
	# 
	# --------------------------------------------------------------



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