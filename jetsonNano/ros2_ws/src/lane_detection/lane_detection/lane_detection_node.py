"""
Based on https://github.com/d-misra/Lane-detection-opencv-python for Kleya's geicar project
"""

# General ROS2 import
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
# Lane Detection Specific Import
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class  lane_detection(Node):
    def __init__(self):
        #Initialization of the node
        super().__init__('lane_detection_node')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image,'image_lane_detection',10)
        self.subscription = self.create_subscription(Image,'image_raw', self.image_raw_callback, 10)
        self.subscription  # prevent unused variable warning
        #Init variable
        self.lanes_coordinates = None
        self.center_camera_px = 640.0 # Value found based on camera photo (cf: notebook)
        
    def image_raw_callback(self, image : Image):
        inputimage = self.bridge.imgmsg_to_cv2(image,desired_encoding='bgr8')
        output, self.lanes_coordinates, _, _ = self.detect_lanes_dark_tape(inputimage,draw_fn=self.draw_lines,thickness=25)
        self.image_with_lanes(output, image) # uncomment to debug

    def image_with_lanes(self, output, image: Image):
        output_image = self.bridge.cv2_to_imgmsg(output,encoding='bgr8')
        output_image.header = image.header
        self.image_pub.publish(output_image)
        
    def error_center_lane(self):
        pass
    
    # Utility functions OpenCV
    
    # Canny edge detection + Remove irrelevant segments of the image and retain only the lane portion
    def canny_edges_with_roi(self, binary_mask, canny_low=50, canny_high=150, roi_vertical_ratio=0.5):
        edges = cv2.Canny(binary_mask, canny_low, canny_high)
        h, w = edges.shape[:2]
        roi_mask = np.zeros_like(edges)
        cv2.rectangle(roi_mask,(0, int(h * roi_vertical_ratio)),(w, h),255,-1)
        return cv2.bitwise_and(edges, roi_mask)

    # Averages Hough segments into left/right lane(s)
    def hough_lines(self,edges,rho=1,theta=np.pi / 180, threshold=25, min_line_length=120,max_line_gap=30):
        return cv2.HoughLinesP(edges, rho=rho,theta=theta, threshold=threshold,minLineLength=min_line_length,maxLineGap=max_line_gap)

    # dark‑tape color mask
    def dark_tape_mask(self,bgr_image,lower_dark=np.array([0, 0, 0]), upper_dark=np.array([180, 80, 120]),kernel_size=5):
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_dark, upper_dark)

        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        return mask

    def pick_lane_lines(self,image_shape, lines, min_slope=0.2):
        if lines is None:
            return None

        _, w = image_shape[:2]
        left, right = [], []

        for l in lines:
            x1, y1, x2, y2 = l.reshape(4)
            if x2 == x1:
                continue
            m = (y2 - y1) / (x2 - x1)
            if abs(m) < min_slope:
                continue
            xm = 0.5 * (x1 + x2)
            if xm < w / 2:
                left.append((x1, y1, x2, y2))
            else:
                right.append((x1, y1, x2, y2))

        lanes = []
        if left:
            lanes.append(np.mean(left, axis=0).astype(int))
        if right:
            lanes.append(np.mean(right, axis=0).astype(int))
        return np.array(lanes) if lanes else None


    def detect_lanes_dark_tape(self, inputimage, draw_fn, thickness=8):
        mask = self.dark_tape_mask(inputimage)
        edges = self.canny_edges_with_roi(mask)
        lines = self.hough_lines(edges)
        lane_lines = self.pick_lane_lines(inputimage.shape, lines)

        output = draw_fn(inputimage, lane_lines, thickness=thickness)
        return output, lane_lines, edges, mask

    # Draws lines of given thickness over an image
    def draw_lines(self,image, lines, thickness):
        if lines is None:
            return image

        line_image = np.zeros_like(image)
        if len(line_image.shape) == 2:
            line_image = cv2.cvtColor(line_image, cv2.COLOR_GRAY2BGR)

        color = (0, 0, 255)
        for x1, y1, x2, y2 in lines:
            cv2.line(line_image, (x1, y1), (x2, y2), color, thickness)

        combined_image = cv2.addWeighted(image, 0.8, line_image, 1.0, 0.0)
        return combined_image

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