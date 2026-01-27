import math
import rclpy
import requests
from rclpy.node import Node
from interfaces.msg import MotorsFeedback, GeneralData, HmiFeatures, HmiStates, MotorsOrder
from std_msgs.msg import Bool

class HmiNode(Node):

    def __init__(self):

        super().__init__('hmi_node')

        # Publishers
        self.publisher_active_features_hmi = self.create_publisher(HmiFeatures, 'active_features_hmi', 10)
        
        # Suscribers
        self.subscription_motors_feedback = self.create_subscription(MotorsFeedback,'motors_feedback', self.motorsfeedback_callback, 10)
        self.subscription_general_data = self.create_subscription(GeneralData,'general_data', self.generaldata_callback, 10)
        self.subscription_hmi_states = self.create_subscription(HmiStates,'hmi_states', self.hmistates_callback, 10)
        self.subscription_motors_order = self.create_subscription(MotorsOrder,'motors_order', self.motorsorder_callback, 10)
       
        # Variables initialisation
        self.left_rear_RPM = 0  
        self.right_rear_RPM = 0
        self.right_rear_pwm = 0
        self.left_rear_pwm = 0
        self.left_speed = 5
        self.right_speed = 5
        self.speed = 5
        self.battery_level  = 0 
        self.temperature  = 0
        self.pressure  = 0
        self.shockdetected = False
        self.RPM = 0

        self.airbag_state  = "None"
        self.collision_state  = "None"
        self.esp_state  = "None"
        self.lca_state  = "None"

        self.active_features = {
            "collision": True,
            "airbag": True,
            "esp": True,
            "lca": True
        }

        self.timer = self.create_timer(0.3, self.send_to_api)
        self.timer2 = self.create_timer(1, self.fetch_active_features)

        self.get_logger().info("hmi_node READY")

    def fetch_active_features(self):
        try:
            r = requests.get("http://localhost:8000/adas", timeout=0.2)
            data = r.json()
            # Update active features
            self.active_features["collision"] = data.get("collision", True)
            self.active_features["airbag"] = data.get("airbag", True)
            self.active_features["esp"] = data.get("esp", True)
            self.active_features["lca"] = data.get("lca", True)
        except Exception as e:
            self.get_logger().warn(f"Failed to fetch ADAS state from FastAPI: {e}")

        # Publish to ROS topic
        msg = HmiFeatures()
        msg.collision_avoidance_active = self.active_features["collision"]
        msg.airbag_active = self.active_features["airbag"]
        msg.esp_active = self.active_features["esp"]
        msg.lca_active = self.active_features["lca"]
        self.publisher_active_features_hmi.publish(msg)

    #Circonference function : 2*pie*rayon
    def circonference(self,rayon):
        return 2*math.pi*rayon

    def motorsfeedback_callback(self, motors_feedback : MotorsFeedback):

        # RPM variables
        self.left_rear_RPM  = motors_feedback.left_rear_speed  
        self.right_rear_RPM   = motors_feedback.right_rear_speed  
        self.RPM = (self.left_rear_RPM + self.right_rear_RPM) / 2

        # Speed variables
        self.left_speed = (self.left_rear_RPM * self.circonference(0.95) * 0.06)
        self.right_speed = (self.right_rear_RPM * self.circonference(0.95) * 0.06)
        self.speed = (self.left_speed+self.right_speed)/2


    def motorsorder_callback(self, motors_order : MotorsOrder):

        # RPM variables
        self.right_rear_pwm  = motors_order.right_rear_pwm  
        self.left_rear_pwm   = motors_order.left_rear_pwm  


    def generaldata_callback(self, general_data : GeneralData):
        # battery, temperature, pressure
        self.battery_level  = general_data.battery_level  
        self.temperature  = general_data.temperature
        self.pressure  = general_data.pressure


    
    def hmistates_callback(self, hmi_states : HmiStates):
        # state des adas temps reel
        self.airbag_state  = hmi_states.states[1]  #index_airbag
        self.collision_state  = hmi_states.states[0] #index collsion
        self.esp_state  = hmi_states.states[2] #index esp
        self.lca_state  = hmi_states.states[4] #index lca


    def send_to_api(self):

        if (self.left_rear_pwm == 50 and self.right_rear_pwm == 50 and self.speed != 0):
            self.speed = 0
            self.RPM = 0
            
        payload = {
            "speed": self.speed,
            "RPM": self.RPM,
            "battery": max(0.0, min(100.0, ((self.battery_level - 8) / (14 - 8)) * 100)),
            "pressure": self.pressure,
            "temperature": self.temperature,
            "airbag_state": self.airbag_state,
            "collision_state": self.collision_state,
            "esp_state": self.esp_state,
            "lca_state": self.lca_state,
        }

        try:
            requests.post(
                "http://localhost:8000/telemetry",
                json=payload,
                timeout=0.2
            )
        except Exception as e:
            self.get_logger().warn(f"API unreachable: {e}")




def main(args=None):
    rclpy.init(args=args)

    hmi_nodes = HmiNode()

    rclpy.spin(hmi_nodes)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hmi_nodes.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()