import rclpy
from rclpy.node import Node
from interfaces.msg import MotorsOrderAdas
#from rcl_interfaces.msg import SetParametersResult


class  lane_centering_assist(Node):
    def __init__(self):
        #Initialization of the node
        super().__init__('lane_centering_assist_node')
        self.publisher_motors_order = self.create_publisher(MotorsOrderAdas, 'motors_order_lca', 10)
        #self.subscription  # prevent unused variable warning


def main(args=None):
    rclpy.init(args=args)

    lane_centering_assist_node = lane_centering_assist()

    rclpy.spin(lane_centering_assist_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lane_centering_assist_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()