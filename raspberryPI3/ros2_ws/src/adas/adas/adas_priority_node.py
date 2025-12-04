import rclpy
from rclpy.node import Node
from interfaces.msg import MotorsOrder, MotorsOrderAdas, HmiFeatures

#Option1
class adas_priority(Node):

    def __init__(self):
        super().__init__('adas_priority')

        # Subscribers 
        self.sub_collision = self.create_subscription(MotorsOrderAdas, 'motors_order_collision', self.collision_callback, 10)
        self.sub_esp = self.create_subscription(MotorsOrderAdas,'motors_order_esp', self.esp_callback, 10)
        self.sub_active_features_hmi = self.create_subscription(HmiFeatures, 'active_features_HMI', self.active_features_HMI_callback, 10)

        # Publishers
        self.pub_final = self.create_publisher(MotorsOrder, 'motors_order', 10)
        self.pub_features_changes = self.create_publisher(HMIFeatures, 'features_changing_motors_order', 10)

        # Last msgs of each topic
        self.last_collision = None   
        self.last_esp = None

        # Active features
        self.collision_avoidance_active_HMI = False
        self.esp_active_HMI = False

        #Features changing motors_order
        self.collision_avoidance_changes = False
        self.esp_changes = False

        self.get_logger().info("adas_priority READY")

    # Collision avoidance callback
    def collision_callback(self, msg):
        self.last_collision = msg
        self.collision_avoidance_changes = self.last_collision.changes # Collision_avoidance did change motors_order or not (to send it to HMI)
        self.publish_decision()

    # ESP callback
    def esp_callback(self, msg):
        self.last_esp = msg
        self.esp_changes = self.last_esp.changes # ESP did change motors_order or not (to send it to HMI)
        self.publish_decision()

    # Active features HMI callback
    def active_features_HMI_callback(self, msg):
        self.collision_avoidance_active_HMI = msg.collision_avoidance
        self.esp_active_HMI = msg.esp



    def publish_decision(self):

        #Priority order to respect: TODO: Add other features in priority list when implemented
        # 1) Collision Avoidance
        # 2) ESP
        # 3) ...
        # Last ) Any msg can be valid (all contain morors_order_raw)
        msg = MotorsOrder()
        if self.last_collision is not None and self.last_collision.changes and self.collision_avoidance_active_HMI:
            msg.right_rear_pwm = self.last_collision.right_rear_pwm
            msg.left_rear_pwm = self.last_collision.left_rear_pwm
            msg.steering_angle = self.last_collision.steering_angle
        
        # TODO: Decomment this part when ESP implemented
        # elif self.last_esp is not None and self.last_esp.changes and self.esp_active_HMI:
        #     msg.right_rear_pwm = self.last_esp.right_rear_pwm
        #     msg.left_rear_pwm = self.last_esp.left_rear_pwm
        #     msg.steering_angle = self.last_esp.steering_angle
        
        else:
            msg.right_rear_pwm = self.last_collision.right_rear_pwm
            msg.left_rear_pwm = self.last_collision.left_rear_pwm
            msg.steering_angle = self.last_collision.steering_angle
        
        if msg in not None:
            self.pub_final.publish(msg)
            changes_msg = HMI_features()
            changes_msg.collision_avoidance = self.collision_avoidance_changes
            changes_msg.airbag = False
            changes_msg.esp = self.esp_changes

            self.pub_features_changes.publish(changes_msg)
        


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
