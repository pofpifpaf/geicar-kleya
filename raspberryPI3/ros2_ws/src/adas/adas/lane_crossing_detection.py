import rclpy
from rclpy.node import Node
from interfaces.msg import DetectedLane
from rcl_interfaces.msg import SetParametersResult


class  lane_crossing_detection(Node):
    def __init__(self):
        #Initialization of the node
        super().__init__('lane_crossing_detection_node')
        #self.subscription  # prevent unused variable warning
        self.subscription = self.create_subscription(DetectedLane,'detected_lanes_position', self.lane_position_callback, 10)

    def lane_position_callback(self,lanes : DetectedLane):
        pass


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