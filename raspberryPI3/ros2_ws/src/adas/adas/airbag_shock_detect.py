import rclpy
from rclpy.node import Node
import json

from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from pathlib import Path


# Chemin vers le dossier où se trouve ce fichier Python
current_dir = Path(__file__).parent

# Chemin vers ton fichier JSON dans le même dossier
data_json = current_dir / "../../../install/hmi/share/hmidata/data.json"


min_linear_acceleration = 2.5

class airbag_shock_detection(Node):

    def __init__(self):
        #Initialization of the shock_detection_node Node
        super().__init__('shock_detection_node')
        self.publisher_ = self.create_publisher(Bool, 'isShockDetected', 10)

        #Subscription to the IMU topic
        self.subscription = self.create_subscription(Imu,'/imu/data', self.imu_callback,10)
        self.subscription  # prevent unused variable warning


    # function that get called when an IMU value is received and send in the topic isShockDetect
    # true if val >= min_linear_acceleration
    def imu_callback(self, msg):

        # Define a variable with the minimal value for shock detection
        
        # get the data  from the imu topic
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y

        # Compare the value with the min we defined
        if (ax >= min_linear_acceleration or 
            ay >= min_linear_acceleration ):
            shock = True

            # log to debug
            self.get_logger().info(f"Accel = ({ax:.2f}, {ay:.2f}), Shock detected")

        else:
            shock = False

        if shock == True:
            #Open data file
            with open(data_json, "r") as f:
                data = json.load(f)
            data_json["AirbagDeployed"] = True

            #Save data
            with open(data_json, "w") as f:
                json.dump(data, f, indent=4)


        # publish the message with shock state
        shock_msg = Bool()
        shock_msg.data = shock
        self.publisher_.publish(shock_msg)




 

def main(args=None):
    rclpy.init(args=args)

    shock_detection_node = airbag_shock_detection()

    rclpy.spin(shock_detection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    shock_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
