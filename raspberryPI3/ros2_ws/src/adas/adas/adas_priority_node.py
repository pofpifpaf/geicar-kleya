import rclpy
from rclpy.node import Node
import time
from interfaces.msg import MotorsOrder, MotorsOrderAdas, HmiFeatures, HmiStates

from rcl_interfaces.msg import SetParametersResult

STOP = 50
NOT_USED_FEATURE = 10
REFRESH_PERIOD = 0.002

INDEX_COLLISION = 0
INDEX_AIRBAG = 1
INDEX_ESP = 2
INDEX_ACC = 3

class adas_priority(Node):

    def __init__(self):
        super().__init__('adas_priority_node')

        # Subscribers 
        self.sub_raw = self.create_subscription(MotorsOrder, 'motors_order_raw', self.motors_order_raw_callback, 10)

        self.sub_collision = self.create_subscription(MotorsOrderAdas, 'motors_order_collision', self.collision_callback, 10)
        self.sub_airbag = self.create_subscription(MotorsOrderAdas, 'motors_order_airbag', self.airbag_callback, 10)
        self.sub_esp = self.create_subscription(MotorsOrderAdas,'motors_order_esp', self.esp_callback, 10)
        self.sub_acc = self.create_subscription(MotorsOrderAdas, 'motors_order_ACC', self.acc_callback, 10)

        self.sub_active_features_hmi = self.create_subscription(HmiFeatures, 'active_features_hmi', self.active_features_HMI_callback, 10)

        # Publishers
        self.pub_final = self.create_publisher(MotorsOrder, 'motors_order', 10)
        self.pub_states = self.create_publisher(HmiStates, 'hmi_states', 10)

        # Parameters
        self.declare_parameter("airbag_priority", 0)
        self.declare_parameter("collision_priority", 1)
        self.declare_parameter("esp_priority", 2)
        self.declare_parameter("acc_priority", 3)

        self.declare_parameter("airbag_block_duration", 10)

        self.priorities = {"collision": self.get_parameter("collision_priority").value,
                         "airbag": self.get_parameter("airbag_priority").value,
                         "esp": self.get_parameter("esp_priority").value,
                         "acc" : self.get_parameter("acc_priority").value}

        self.airbag_block_duration = self.get_parameter("airbag_block_duration").value

        self.last_offsets = {}
        self.hmi_active = {"collision":True,
                             "airbag":True,
                             "esp":True,
                             "acc": True}

        self.states = {"collision": "state_nothing",
                         "airbag": "state_nothing",
                         "esp": "state_nothing",
                         "acc": "state_nothing"}

        # Active features on HMI
        self.collision_avoidance_active_HMI = 0
        self.esp_active_HMI = 0
        self.airbag_active_HMI = 0

        #Raw motor order
        self.motor_right_rear_pwm = 50
        self.motor_left_rear_pwm = 50
        self.steering_angle = 0

        self.state_airbag_deployed = False
        self.time_airbag_deployed = 0

        self.add_on_set_parameters_callback(self.param_callback)

        self.timer = self.create_timer(REFRESH_PERIOD, self.publish_motors_order_decision)

        self.get_logger().info("node adas_priority READY")

    # Parameters callback
    def param_callback(self, params):
        for p in params:
            if p.name == "airbag_block_duration":
                self.airbag_block_duration = p.value
            if p.name in self.priorities:
                self.priorities[p.name] = p.value
        return SetParametersResult(successful=True)

    # Collision avoidance callback
    def collision_callback(self, msg):
        self.last_offsets["collision"] = msg
        self.states["collision"] = msg.state

    # ESP callback
    def esp_callback(self, msg):
        self.last_offsets["esp"] = msg
        self.states["esp"] = msg.state

    #AIRBAG callback
    def airbag_callback(self, msg):
        self.last_offsets["airbag"] = msg
        self.states["airbag"] = msg.state

    def acc_callback(self, msg):
        self.last_offsets["acc"] = msg
        self.states["acc"] = msg.state
        
    # Active features HMI callback
    def active_features_HMI_callback(self, msg):
        self.hmi_active = {"collision":msg.collision_avoidance_active, "airbag":msg.airbag_active, "esp":msg.esp_active}

    def motors_order_raw_callback(self, msg):
        self.motor_left_rear_pwm = msg.left_rear_pwm
        self.motor_right_rear_pwm = msg.right_rear_pwm
        self.steering_angle = msg.steering_angle

    def publish_motors_order_decision(self):
        
        min_prio = 50

        for key, msg in self.last_offsets.items():

            if msg.active and self.hmi_active[key]:

                min_prio = min(self.priorities[key], min_prio)
                
                if (self.priorities[key] == min_prio):

                    if msg.emergency_stop and key == "airbag":

                        self.motor_left_rear_pwm = STOP
                        self.motor_right_rear_pwm = STOP

                        if not self.state_airbag_deployed:
                            self.get_logger().info(f"Emergency stop detected from airbag node")
                            self.time_airbag_deployed = time.time()
                            self.state_airbag_deployed = True


                    elif msg.emergency_stop:

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

        if (time.time() - self.time_airbag_deployed < self.airbag_block_duration) and self.state_airbag_deployed:
            # self.get_logger().info(f"airbag deployed for time = {time.time() - self.time_airbag_deployed}")
            motors_msg.right_rear_pwm = STOP
            motors_msg.left_rear_pwm = STOP
            motors_msg.steering_angle = self.steering_angle
        else:
            motors_msg.right_rear_pwm = self.motor_right_rear_pwm
            motors_msg.left_rear_pwm = self.motor_left_rear_pwm
            motors_msg.steering_angle = self.steering_angle
            self.state_airbag_deployed = False
            
        self.pub_final.publish(motors_msg)

        states_msg = HmiStates()

        states_msg.states[INDEX_COLLISION] = self.states["collision"]
        if (self.state_airbag_deployed):
            states_msg.states[INDEX_AIRBAG] = "state_deployed"
        else:
            states_msg.states[INDEX_AIRBAG] = "state_nothing" 
        states_msg.states[INDEX_ESP] = self.states["esp"]      
        
        self.pub_states.publish(states_msg)

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