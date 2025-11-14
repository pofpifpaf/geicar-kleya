import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

class DetectShock(Node):

    def __init__(self):
        #Initialization of the shockDetection Node
        super().__init__('shock_detection')
        self.publisher_ = self.create_publisher(Bool, 'isShockDetected', 10)

        #Subscription to the IMU topic
        self.subscription = self.create_subscription(Imu,'/imu/data', self.imu_callback,10)
        self.subscription  # prevent unused variable warning


    # function that get called when an IMU value is received and send in the topic isShockDetect
    # true if val >= min_linear_acceleration
    def imu_callback(self, msg):

        # Define a variable with the minimal value for shock detection
        min_linear_acceleration = 1.5
        
        # get the data  from the imu topic
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Compare the value with the min we defined
        if (ax >= min_linear_acceleration or 
            ay >= min_linear_acceleration or 
            az >= min_linear_acceleration):
            shock = True
        else:
            shock = False
        # publish the message with shock state
        shock_msg = Bool()
        shock_msg.data = shock
        self.publisher_.publish(shock_msg)

        # log to debug
        self.get_logger().info(f"Accel = ({ax:.2f}, {ay:.2f}, {az:.2f}), Shock Status = {shock_msg.data}")



 

def main(args=None):
    rclpy.init(args=args)

    shockDetection = DetectShock()

    rclpy.spin(shockDetection)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    shockDetection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
