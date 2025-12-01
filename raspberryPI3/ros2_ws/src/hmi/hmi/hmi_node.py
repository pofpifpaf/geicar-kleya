import math
import rclpy
import json
from rclpy.node import Node

from interfaces.msg import MotorsFeedback, GeneralData

from pathlib import Path

# Chemin vers le dossier où se trouve ce fichier Python
current_dir = Path(__file__).parent

# Chemin vers ton fichier JSON dans le même dossier
data_json = current_dir / "../../../../share/hmidata/data.json"

class hmi_node(Node):

    def __init__(self):

        super().__init__('hmi_node')
        # on doit publish dans le topic de control_activation
        #self.publisher_motors_order = self.create_publisher(MotorsOrder, 'motors_order', 10)

        self.subscription = self.create_subscription(MotorsFeedback,'motors_feedback', self.motorsfeedback_callback, 10)
        self.subscription = self.create_subscription(GeneralData,'general_data', self.generaldata_callback, 10)
    
        self.left_rear_RPM  = 0  
        self.right_rear_RPM   = 0
        self.left_speed = 5
        self.right_speed = 5
        self.speed = 5
        self.battery_level  = 0 
        self.temperature  = 0
        self.pressure  = 0

        self.get_logger().info("hmi_node READY")

    #Circonference function : 2*pie*rayon
    def circonference(self,rayon):
        return 2*math.pi*rayon

    def motorsfeedback_callback(self, motors_feedback : MotorsFeedback):

        # RPM variables
        self.left_rear_RPM  = motors_feedback.left_rear_speed  
        self.right_rear_RPM   = motors_feedback.right_rear_speed  

        # Speed variables
        self.left_speed = (self.left_rear_RPM * self.circonference(0.95) * 0.06)
        self.right_speed = (self.right_rear_RPM * self.circonference(0.95) * 0.06)
        self.speed = (self.left_speed+self.right_speed)/2

        self.save_datas()


    def generaldata_callback(self, general_data : GeneralData):
        # battery, temperature, pressure
        self.battery_level  = general_data.battery_level  
        self.temperature  = general_data.temperature
        self.pressure  = general_data.pressure

        self.save_datas()


    def save_datas(self):
        
        #Open data file
        with open(data_json, "r") as f:
            data = json.load(f)
         
        #Write data
        data["battery"] = self.battery_level
        data["pressure"] = self.pressure
        data["temperature"] = self.temperature
        data["speed"] = self.speed
        
        #Save data
        with open(data_json, "w") as f:
            json.dump(data, f, indent=4)

        #self.publisher_motors_order.publish(msg)




def main(args=None):
    rclpy.init(args=args)

    hmi_nodes = hmi_node()

    rclpy.spin(hmi_nodes)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hmi_nodes.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()