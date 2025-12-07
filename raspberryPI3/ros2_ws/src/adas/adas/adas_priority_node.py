import rclpy
from rclpy.node import Node
from interfaces.msg import MotorsOrder, MotorsOrderAdas, HmiFeatures


#Priorities
COLLISION_AVOIDANCE_PRIO = 1
AIRBAG_PRIO = 1
ESP_PRIO = 2

NOT_USED_FEATURE = 10



class adas_priority(Node):

    def __init__(self):
        super().__init__('adas_priority')

        # Subscribers 
        self.sub_collision = self.create_subscription(MotorsOrderAdas, 'motors_order_collision', self.collision_callback, 10)
        self.sub_esp = self.create_subscription(MotorsOrderAdas,'motors_order_esp', self.esp_callback, 10)
        self.sub_active_features_hmi = self.create_subscription(HmiFeatures, 'active_features_hmi', self.active_features_HMI_callback, 10)
        self.sub_airbag = self.create_subscription(Bool, 'isShockDetected', self.airbag_callback, 10)

        # Publishers
        self.pub_final = self.create_publisher(MotorsOrder, 'motors_order', 10)
        self.pub_features_priorities = self.create_publisher(HmiFeatures, 'features_priority_hmi', 10)

        # Last msgs of each topic
        self.last_collision = None   
        self.last_esp = None

        # Active features on HMI
        self.collision_avoidance_active_HMI = 0
        self.esp_active_HMI = 0
        self.airbag_active_HMI = 0

        #Features changing motors_order
        self.collision_priority = COLLISION_AVOIDANCE_PRIO
        self.esp_priority = ESP_PRIO
        self.airbag_priority = AIRBAG_PRIO

        self.get_logger().info("adas_priority READY")

    # Collision avoidance callback
    def collision_callback(self, msg):
        self.last_collision = msg
        if msg.changes and self.collision_avoidance_active_HMI:
            self.collision_priority = COLLISION_AVOIDANCE_PRIO
        else:
            self.collision_priority = NOT_USED_FEATURE

        self.publish_motors_order_decision()

    # ESP callback
    def esp_callback(self, msg):
        self.last_esp = msg
        if msg.changes and self.esp_active_HMI:
            self.esp_priority = ESP_PRIO 
        else:
            self.esp_priority = NOT_USED_FEATURE
        
        self.publish_motors_order_decision()

    #AIRBAG callback
    def airbag_callback(self, msg):
        if msg and self.airbag_active_HMI:
            self.airbag_priority = AIRBAG_PRIO
        else:
            self.airbag_priority = NOT_USED_FEATURE
        
    # Active features HMI callback
    def active_features_HMI_callback(self, msg):
        self.collision_avoidance_active_HMI = msg.collision_avoidance
        self.esp_active_HMI = msg.esp
        self.airbag_active_HMI = msg.airbag


    def publish_motors_order_decision(self):
        candidates = []

        if self.last_collision is not None:
            candidates.append((self.collision_priority, self.last_collision))

        if self.last_esp is not None:
            candidates.append((self.esp_priority, self.last_esp))

        if not candidates:
            return

        selected_priority, selected_msg = min(candidates, key=lambda x: x[0])

        msg = MotorsOrder()
        msg.right_rear_pwm = selected_msg.right_rear_pwm
        msg.left_rear_pwm = selected_msg.left_rear_pwm
        msg.steering_angle = selected_msg.steering_angle

        self.pub_final.publish(msg)

        # Publishing priorities for HMI use
        prio_msg = HmiFeatures()
        prio_msg.collision_avoidance = self.collision_priority
        prio_msg.airbag = self.airbag_priority
        prio_msg.esp = self.esp_priority

        self.pub_features_priorities.publish(prio_msg)

            
        


def main(args=None):
    rclpy.init(args=args)
    node = adas_priority()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
