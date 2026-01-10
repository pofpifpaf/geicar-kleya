"""
Based on https://github.com/d-misra/Lane-detection-opencv-python for Kleya's geicar project
"""

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

        output, self.lanes_coordinates, _, _ = self.detect_lanes_dark_tape(inputimage,draw_fn=self.draw_lines,thickness=5)
        self.image_with_lanes(output, image) # uncomment to debug

    def image_with_lanes(self, output, image: CompressedImage):
        msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
        msg.header = image.header
        self.image_pub.publish(msg)

        
    def error_center_lane(self):
        pass
    
    # Utility functions OpenCV
    
    # Canny edge detection + Remove irrelevant segments of the image and retain only the lane portion
    def canny_edges_with_roi(self, binary_mask, canny_low=50, canny_high=150, roi_vertical_ratio=0.55):
        edges = cv2.Canny(binary_mask, canny_low, canny_high)
        h, w = edges.shape[:2]
        roi_mask = np.zeros_like(edges)
        cv2.rectangle(roi_mask,(0, int(h * roi_vertical_ratio)),(w, h),255,-1)
        return cv2.bitwise_and(edges, roi_mask)

    # Averages Hough segments into left/right lane(s)
    def hough_lines(self,edges,rho=1,theta=np.pi / 180, threshold=15, min_line_length=80,max_line_gap=30):
        return cv2.HoughLinesP(edges, rho=rho,theta=theta, threshold=threshold,minLineLength=min_line_length,maxLineGap=max_line_gap)
    
    # Dark Mask Function using YUCrCb
    def dark_tape_mask(self, bgr_image,lower_ycrcb=np.array([0, 100, 100]),upper_ycrcb=np.array([110, 140, 140]),kernel_size=5): # TO DO: Modify the thereshold based on rosbag in the corridor
        # Code Hexa couleur via le couloir: #656450 pour le low et #1d1e10 pour le high
        # Conversion to YCrCb
        ycrcb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2YCrCb)
        # Mask threshold
        mask = cv2.inRange(ycrcb, lower_ycrcb, upper_ycrcb)
        # Morphological filtering
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        return mask

    def pick_lane_lines(self,image_shape, lines,min_angle_deg=3, max_angle_deg=85,k_longest=8,min_length=20):   #Value to modify  
        if lines is None:
            return None

        h, w = image_shape[:2]
        x_center = w / 2.0
        min_angle = np.deg2rad(min_angle_deg)
        max_angle = np.deg2rad(max_angle_deg)

        segs = []
        for l in lines:
            x1, y1, x2, y2 = l.reshape(4)
            dx, dy = x2 - x1, y2 - y1
            length = np.hypot(dx, dy)
            if length < min_length:
                continue
            angle = np.arctan2(dy, dx)
            aabs = abs(angle)
            if aabs < min_angle or aabs > max_angle:
                continue
            xm = 0.5 * (x1 + x2)
            side = 'L' if xm < x_center else 'R'
            segs.append({'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                         'length': length, 'angle': angle,
                         'xm': xm, 'side': side})

        if len(segs) == 0:
            return None

        left  = sorted([s for s in segs if s['side']=='L'],
                    key=lambda s: -s['length'])[:k_longest]
        right = sorted([s for s in segs if s['side']=='R'],
                    key=lambda s: -s['length'])[:k_longest]

        def avg_side(group):
            arr = np.array([[g['x1'], g['y1'], g['x2'], g['y2']] for g in group],
                        dtype=float)
            return arr.mean(axis=0).astype(int)

        lanes = []

        if left and right:
            # Geometry scoring to pick one best (L,R) pair
            y_ref = 0.75 * h
            best_pair, best_score = None, -1e9

            def x_at_y(seg, y):
                x1, y1, x2, y2 = seg['x1'], seg['y1'], seg['x2'], seg['y2']
                if y2 == y1:
                    return (x1 + x2) / 2.0
                return x1 + (y - y1) * (x2 - x1) / (y2 - y1)

            for L in left:
                for R in right:
                    # Parallelism
                    dang = abs(L['angle'] - R['angle'])
                    dang = min(dang, np.pi - dang)
                    score_parallel = -dang

                    # Width near bottom
                    x_left = x_at_y(L, y_ref)
                    x_right = x_at_y(R, y_ref)
                    width = x_right - x_left
                    #if width <= 80 or width >= 0.8 * w:
                    if width <= 40 or width >= 0.9 * w:
                        continue

                    # Centering
                    lane_center = (x_left + x_right) / 2.0
                    off_center = abs(lane_center - x_center)
                    score_center = -off_center / w

                    # Length
                    score_len = (L['length'] + R['length']) / (h + w)

                    score = 3.0*score_parallel + 2.0*score_len + 1.0*score_center
                    if score > best_score:
                        best_score = score
                        best_pair = (L, R)

            if best_pair is not None:
                L, R = best_pair
                lanes.append(avg_side([L]))
                lanes.append(avg_side([R]))

            else:
                # best single segment overall
                best_single = max(segs, key=lambda s: s['length'])
                lanes.append(avg_side([best_single]))

        elif left and not right:
            lanes.append(avg_side(left))   # only left lane

        elif right and not left:
            lanes.append(avg_side(right))  # only right lane

        return np.array(lanes) if lanes else None

    def detect_lanes_dark_tape(self,input_image, draw_fn, thickness=8, debug=False):
        mask = self.dark_tape_mask(input_image)
        edges = self.canny_edges_with_roi(mask)

        edges = cv2.dilate(edges, np.ones((3,3), np.uint8), iterations=1)
        lines = self.hough_lines(edges)
        lane_lines = self.pick_lane_lines(input_image.shape, lines)

        output = draw_fn(input_image, lane_lines, thickness=thickness)

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