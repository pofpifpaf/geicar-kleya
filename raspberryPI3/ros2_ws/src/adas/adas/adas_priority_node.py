import rclpy
from rclpy.node import Node
from interfaces.msg import MotorsOrder, MotorsOrderAdas, HmiFeatures

#Priorities
COLLISION_AVOIDANCE_PRIO = 1
AIRBAG_PRIO = 0
ESP_PRIO = 2

NOT_USED_FEATURE = 10

STOP = 50

REFRESH_PERIOD = 0.002

class adas_priority(Node):

    def __init__(self):
        super().__init__('adas_priority_node')

        # Subscribers 
        self.sub_raw = self.create_subscription(MotorsOrder, 'motors_order_raw', self.motors_order_raw_callback, 10)

        self.sub_collision = self.create_subscription(MotorsOrderAdas, 'motors_order_collision', self.collision_callback, 10)
        self.sub_airbag = self.create_subscription(MotorsOrderAdas, 'motors_order_airbag', self.airbag_callback, 10)
        self.sub_esp = self.create_subscription(MotorsOrderAdas,'motors_order_esp', self.esp_callback, 10)

        self.sub_active_features_hmi = self.create_subscription(HmiFeatures, 'active_features_hmi', self.active_features_HMI_callback, 10)

        # Publishers
        self.pub_final = self.create_publisher(MotorsOrder, 'motors_order', 10)

        self.last_offsets = {}
        self.priorities = {"collision":COLLISION_AVOIDANCE_PRIO,
                         "airbag":AIRBAG_PRIO,
                         "esp":ESP_PRIO}

        self.hmi_priorities = {}

        # Active features on HMI
        self.collision_avoidance_active_HMI = 0
        self.esp_active_HMI = 0
        self.airbag_active_HMI = 0

        #Raw motor order
        self.motor_right_rear_pwm = 50
        self.motor_left_rear_pwm = 50
        self.steering_angle = 0

        self.timer = self.create_timer(REFRESH_PERIOD, self.publish_motors_order_decision)

        self.get_logger().info("node adas_priority READY")

    # Collision avoidance callback
    def collision_callback(self, msg):
        # self.last_collision = msg
        self.last_offsets["collision"] = msg

    # ESP callback
    def esp_callback(self, msg):
        # self.last_esp = msg
        self.last_offsets["esp"] = msg

    #AIRBAG callback
    def airbag_callback(self, msg):
        # self.last_airbag = msg
        self.last_offsets["airbag"] = msg
        
    # Active features HMI callback
    def active_features_HMI_callback(self, msg):
        self.hmi_priorities = {"collision":msg.collision_avoidance_active, "airbag":msg.airbag_active, "esp":msg.esp_active}

    def motors_order_raw_callback(self, msg):
        self.motor_left_rear_pwm = msg.motor_left_rear_pwm
        self.motor_right_rear_pwm = msg.motor_right_rear_pwm
        self.steering_angle = msg.steering_angle

    def publish_motors_order_decision(self):
        
        min_prio = 50

        for key, msg in self.last_offsets.items():

            if msg.active and hmi_priorities[key]:

                min_prio = min(self.priorities[key], min_prio)
                
                if (self.priorities[key] == min_prio):

                    if emergency_stop and key == "airbag":

                        self.motor_left_rear_pwm = STOP
                        self.motor_right_rear_pwm = STOP

                        self.get_logger().info(f"Emergency stop detected from {key} node")

                    elif emergency_stop:

                        if self.motor_right_rear_pwm > STOP and self.motor_left_rear_pwm > STOP:

                            self.motor_left_rear_pwm = STOP
                            self.motor_right_rear_pwm = STOP

                    else:
                        self.motor_left_rear_pwm = max(min(msg.offset_left_rear_pwm + self.motor_left_rear_pwm, 100), 0)
                        self.motor_right_rear_pwm = max(min(msg.offset_right_rear_pwm + self.motor_right_rear_pwm, 100), 0)

                        self.steering_angle = max(min(msg.offset_steering_angle + self.steering_angle, 127), -128)

                        if self.motor_right_rear_pwm > STOP and self.motor_left_rear_pwm > STOP:

                            self.motor_right_rear_pwm = min(self.motor_left_rear_pwm, msg.max_pwm + STOP)
                            self.motor_right_rear_pwm = min(self.motor_right_rear_pwm, msg.max_pwm + STOP)

        motors_msg = MotorsOrder()
        motors_msg.right_rear_pwm = self.motor_right_rear_pwm
        motors_msg.left_rear_pwm = self.motor_left_rear_pwm
        motors_msg.steering_angle = self.steering_angle

        self.pub_final.publish(motors_msg)        
        

def main(args=None):
    rclpy.init(args=args)
    adas_priority_node = adas_priority()
    rclpy.spin(adas_priority_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    adas_priority_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
