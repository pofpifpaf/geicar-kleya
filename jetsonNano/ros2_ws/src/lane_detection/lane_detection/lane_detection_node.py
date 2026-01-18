
# General ROS2 import
import rclpy
from rclpy.node import Node
# Lane Detection Specific Import
from sensor_msgs.msg import CompressedImage, Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class  lane_detection(Node):
    def __init__(self):
        #Initialization of the node
        super().__init__('lane_detection_node')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image,'image_lane_detection',10)
        self.subscription = self.create_subscription(CompressedImage,'image_raw/compressed', self.image_raw_callback, 10)
        self.subscription  # prevent unused variable warning
        #Init variable
        self.lanes_coordinates = None
        self.center_camera_px = 640.0 # Value found based on camera photo (cf: notebook)
        
    def image_raw_callback(self, image : CompressedImage):
        # Convertir le CompressedImage en array numpy
        np_arr = np.frombuffer(image.data, np.uint8)
        inputimage = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV BGR

        output, self.lanes_coordinates, _, _ = self.detect_lanes_dark_tape(inputimage)
        self.image_with_lanes(output, image) # uncomment to debug

    def image_with_lanes(self, output, image: CompressedImage):
        msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
        msg.header = image.header
        self.image_pub.publish(msg)


    # Utility functions OpenCV
    # Cut the irrelevant part of the picture 
    def roi_on_mask(self, mask, roi_top_ratio=0.55, roi_bottom_ratio=0.05):
        h, w = mask.shape[:2]
        roi_mask = np.zeros_like(mask)
        roi_top = int(h * roi_top_ratio)
        roi_bottom = int(h * (1 - roi_bottom_ratio))
        cv2.rectangle(roi_mask, (0, roi_top), (w, roi_bottom), 255, -1)
        return cv2.bitwise_and(mask, roi_mask)

    def split_left_right_edges(self, edges, min_branch_length=40):
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
    def dark_tape_mask(self, bgr_image, kernel_size=5):
        ycrcb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2YCrCb)
        # Range 1: Optimal corridor
        # Code Hexa couleur via le couloir: #656450 pour le low et #1d1e10 pour le high
        range1_low  = np.array([0,   100,  100])
        range1_high = np.array([110, 140, 140])
        mask1 = cv2.inRange(ycrcb, range1_low, range1_high)
        
        # Range 2: Lighter environment  
        range2_low  = np.array([0,   110,  110])  
        range2_high = np.array([125, 155, 155])
        mask2 = cv2.inRange(ycrcb, range2_low, range2_high)
        
        # Combine: OR masks
        mask = cv2.bitwise_or(mask1, mask2)
        
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        return mask

    def centerline_from_mask(self, mask, kernel_size=3):
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        eroded = cv2.erode(mask, kernel, iterations=1)
        
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

    def prune_by_length(self, skel, min_length=60):
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(skel, connectivity=8)
        pruned = np.zeros_like(skel)
        for i in range(1, num_labels):
            width = stats[i, cv2.CC_STAT_WIDTH]
            height = stats[i, cv2.CC_STAT_HEIGHT]
            length_proxy = max(width, height)
            if length_proxy >= min_length:
                pruned[labels == i] = 255
        return pruned

    def polynomial_fit_from_points(self, ys, xs, min_points=50, min_span=40):
        if xs is None or len(xs) < min_points or (len(ys) > 0 and ys.max() - ys.min() < min_span):
            return None
        sort_idx = np.argsort(ys)
        ys_sorted, xs_sorted = ys[sort_idx], xs[sort_idx]
        trim = max(1, int(0.1 * len(ys)))
        clean_ys, clean_xs = ys_sorted[trim:-trim], xs_sorted[trim:-trim]
        coeffs = np.polyfit(clean_ys, clean_xs, 1) # Affine function
        return coeffs

    def detect_lanes_dark_tape(self, input_image, draw_fn=None, thickness=8, debug=False):
        mask = self.dark_tape_mask(input_image)
        mask_skel = self.centerline_from_mask(mask)
        mask_skel = self.prune_by_length(mask_skel)
        roi_skel = self.roi_on_mask(mask_skel)
        (left_y, left_x), (right_y, right_x) = self.split_left_right_edges(roi_skel)
        
        left_strength = len(left_x) if left_x is not None else 0
        right_strength = len(right_x) if right_x is not None else 0
        
        left_coeffs = self.polynomial_fit_from_points(left_y, left_x) if left_strength >= 50 else None
        right_coeffs = self.polynomial_fit_from_points(right_y, right_x) if right_strength >= 50 else None
        
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