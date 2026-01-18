
# General ROS2 import
import rclpy
from rclpy.node import Node
# Lane Detection Specific Import
from sensor_msgs.msg import CompressedImage, Image
from interfaces.msg import DetectedLane
import cv2
from cv_bridge import CvBridge
import numpy as np

class  lane_detection(Node):
    # Constant to tune for lane detection
    MIN_BRANCH_LENGTH = 320      # min area for connected components
    MIN_POLY_POINTS = 180       # min points for polynomial fit
    PRUNE_MIN_LENGTH = 120       # min skeleton length
    MIN_LANE_STRENGTH = 300     # min number of points in branch to be valid
    GAUSSIAN_BLUR_CONST = 5
    KERNEL_MASK_VALUE = 3
    def __init__(self):
        #Initialization of the node
        super().__init__('lane_detection_node')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image,'image_lane_detection',10)
        self.lane_pub = self.create_publisher(DetectedLane, 'detected_lanes_position', 10)
        self.subscription = self.create_subscription(CompressedImage,'image_raw/compressed', self.image_raw_callback, 10)
        self.subscription  # prevent unused variable warning
        #Init variable
        self.lanes_coordinates = None
        self.center_camera_px = 640.0 # Value found based on camera photo (cf: notebook)
        
    def image_raw_callback(self, image : CompressedImage):
        # Convertir le CompressedImage en array numpy
        np_arr = np.frombuffer(image.data, np.uint8)
        inputimage = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV BGR

        output , self.lanes_coordinates, _, _ = self.detect_lanes_dark_tape(inputimage)
        self.image_with_lanes(output, image) # uncomment to debug
        
        left_coeffs, right_coeffs = self.lanes_coordinates
        h = inputimage.shape[0]
        
        # Create the message to send over ros2 topic
        lane_msg = DetectedLane()
        left_msg = self.lane_msg_from_coeffs(left_coeffs, h)
        right_msg = self.lane_msg_from_coeffs(right_coeffs, h)

        lane_msg.lane1_bottom_x = left_msg['bottom_x']
        lane_msg.lane1_bottom_y = left_msg['bottom_y']
        lane_msg.lane1_top_x = left_msg['top_x']
        lane_msg.lane1_top_y = left_msg['top_y']
        lane_msg.lane1_valid = left_msg['valid']

        lane_msg.lane2_bottom_x = right_msg['bottom_x']
        lane_msg.lane2_bottom_y = right_msg['bottom_y']
        lane_msg.lane2_top_x = right_msg['top_x']
        lane_msg.lane2_top_y = right_msg['top_y']
        lane_msg.lane2_valid = right_msg['valid']

        self.lane_pub.publish(lane_msg)

    def image_with_lanes(self, output, image: CompressedImage):
        msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
        msg.header = image.header
        self.image_pub.publish(msg)

    # Function to create the LaneDetection Msg
    def lane_msg_from_coeffs(self, coeffs, h, y_min_ratio=0.55):
        msg = {}
        if coeffs is not None:
            b, c = coeffs
            y_min = int(h * y_min_ratio)
            y_max = h
            x_min = int(b * y_min + c)
            x_max = int(b * y_max + c)
            msg['bottom_x'] = x_max
            msg['bottom_y'] = y_max
            msg['top_x'] = x_min
            msg['top_y'] = y_min
            msg['valid'] = True
        else:
            msg['bottom_x'] = msg['bottom_y'] = 0
            msg['top_x'] = msg['top_y'] = 0
            msg['valid'] = False
        return msg

    # Utility functions OpenCV
    # Cut the irrelevant part of the picture 
    def roi_on_mask(self, mask,roi_top_ratio=0.55, roi_bottom_ratio=0.2,roi_left_ratio=0.25,roi_right_ratio=0.25):

        h, w = mask.shape[:2]
        roi_mask = np.zeros_like(mask)

        top = int(h * roi_top_ratio)
        bottom = int(h * (1 - roi_bottom_ratio))
        left = int(w * roi_left_ratio)
        right = int(w * (1 - roi_right_ratio))

        cv2.rectangle(roi_mask, (left, top), (right, bottom), 255, -1)
        return cv2.bitwise_and(mask, roi_mask)


    def split_left_right_edges(self, edges, min_branch_length=MIN_BRANCH_LENGTH):
        ys, xs = np.nonzero(edges > 0)
        if len(xs) == 0:
            return (None, None), (None, None)
        h, w = edges.shape[:2]
        x_mid = w / 2.0
        left_mask = xs < x_mid
        right_mask = xs >= x_mid
        left_y, left_x = self.longest_branch(ys[left_mask], xs[left_mask], min_branch_length, h, w)
        right_y, right_x = self.longest_branch(ys[right_mask], xs[right_mask], min_branch_length, h, w)
        return (left_y, left_x), (right_y, right_x)

    # Function to filter when severals branches are detected
    def longest_branch(self, ys, xs, min_length, h, w):
        if len(xs) == 0:
            return None, None
        ys_clipped = np.clip(ys.astype(int), 0, h-1)
        xs_clipped = np.clip(xs.astype(int), 0, w-1)
        temp = np.zeros((h, w), np.uint8)
        temp[ys_clipped, xs_clipped] = 255
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(temp, connectivity=8)
        if num_labels > 1:
            largest_idx = np.argmax(stats[1:, cv2.CC_STAT_AREA]) + 1
            if stats[largest_idx, cv2.CC_STAT_AREA] >= min_length:
                comp_ys, comp_xs = np.nonzero(labels == largest_idx)
                return comp_ys, comp_xs
        return None, None

    # Dark Mask Function using YUCrCb
    def dark_tape_mask(self,bgr_image, kernel_size=KERNEL_MASK_VALUE, clip_limit=2.0, tile_grid_size=(8,8)):
        #Adaptive dark-tape mask using CLAHE on the Y channel.
        # Convert to YCrCb and extract Y channel
        ycrcb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2YCrCb)
        Y, Cr, Cb = cv2.split(ycrcb)
        # Apply CLAHE (adaptive histogram equalization) to enhance dark tape
        clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
        chan_y = clahe.apply(Y)
        # Simple adaptive threshold: mean - some factor
        mean_y = np.mean(chan_y)
        thresh = max(0, int(mean_y * 0.4))  # dark tape is darker than mean
        # Mask: pixels darker than threshold
        mask = cv2.inRange(chan_y, 0, thresh)
        # Morphology to clean noise
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        return mask


    def centerline_from_mask(self, mask, kernel_size=3):
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        eroded = cv2.erode(mask, kernel, iterations=1)
        # Test putting blur to reduce noise
        eroded = cv2.GaussianBlur(eroded, (self.GAUSSIAN_BLUR_CONST, self.GAUSSIAN_BLUR_CONST), 0)  # soft blur
        skel = np.zeros(mask.shape, np.uint8)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
        done = False
        while not done:
            eroded_temp = cv2.erode(eroded, element)
            temp = cv2.dilate(eroded_temp, element)
            temp = cv2.subtract(eroded, temp)
            skel = cv2.bitwise_or(skel, temp)
            eroded = eroded_temp.copy()
            if cv2.countNonZero(eroded) == 0:
                done = True
        return skel

    def prune_by_length(self, skel, min_length=PRUNE_MIN_LENGTH):
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(skel, connectivity=8)
        pruned = np.zeros_like(skel)
        for i in range(1, num_labels):
            width = stats[i, cv2.CC_STAT_WIDTH]
            height = stats[i, cv2.CC_STAT_HEIGHT]
            length_proxy = max(width, height)
            if length_proxy >= min_length:
                pruned[labels == i] = 255
        return pruned

    def polynomial_fit_from_points(self, ys, xs, min_points=MIN_POLY_POINTS, min_span=80):
        if xs is None or len(xs) < min_points or (len(ys) > 0 and ys.max() - ys.min() < min_span):
            return None
        sort_idx = np.argsort(ys)
        ys_sorted, xs_sorted = ys[sort_idx], xs[sort_idx]
        trim = max(1, int(0.1 * len(ys)))
        clean_ys, clean_xs = ys_sorted[trim:-trim], xs_sorted[trim:-trim]
        coeffs = np.polyfit(clean_ys, clean_xs, 1) # Affine function
        return coeffs

    def detect_lanes_dark_tape(self, input_image, draw_fn=None, thickness=8, min_lane_strength=MIN_LANE_STRENGTH):
        mask = self.dark_tape_mask(input_image)
        mask_skel = self.centerline_from_mask(mask)
        mask_skel = self.prune_by_length(mask_skel)
        roi_skel = self.roi_on_mask(mask_skel)
        (left_y, left_x), (right_y, right_x) = self.split_left_right_edges(roi_skel)
        
        left_strength = len(left_x) if left_x is not None else 0
        right_strength = len(right_x) if right_x is not None else 0
        
        left_coeffs = self.polynomial_fit_from_points(left_y, left_x) if left_strength >= min_lane_strength else None
        right_coeffs = self.polynomial_fit_from_points(right_y, right_x) if right_strength >= min_lane_strength else None
        
        self.lanes_coordinates = (left_coeffs, right_coeffs)  # For steering
        
        output = self.draw_lines(input_image, left_coeffs, right_coeffs, thickness)
        return output, self.lanes_coordinates, roi_skel, mask

    # Draws lines of given thickness over an image
    def draw_lines(self,image, left_coeffs=None, right_coeffs=None, thickness=8, color=(0, 0, 255), y_min_ratio=0.55):

        line_image = np.zeros_like(image)
        h = image.shape[0]
        y_min = int(h * y_min_ratio)
        y_max = h

        for coeffs in [left_coeffs, right_coeffs]:
            if coeffs is not None:
                b, c = coeffs
                x1 = int(b*y_min + c)
                x2 = int(b*y_max + c)
                cv2.line(line_image, (x1, y_min), (x2, y_max), color, thickness)

        return cv2.addWeighted(image, 0.8, line_image, 1.0, 0.0)

def main(args=None):
    rclpy.init(args=args)

    lane_detection_node = lane_detection()

    rclpy.spin(lane_detection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lane_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()