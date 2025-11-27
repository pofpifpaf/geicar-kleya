import math
import rclpy
from rclpy.node import Node

from interfaces.msg import Ultrasonic, MotorsOrder


class hmi_node(Node):

    def __init__(self):

        super().__init__('hmi_node')
        # on doit publish dans le topic de control_activation
        #self.publisher_motors_order = self.create_publisher(MotorsOrder, 'motors_order', 10)

        self.subscription = self.create_subscription(MotorsFeedback,'motors_feedback', self.motorsfeedback_callback, 10)
        self.subscription = self.create_subscription(GeneralData,'general_data', self.generaldata_callback, 10)

        self.get_logger().info("hmi_node READY")

    def circonference(self,rayon):
        return 2*math.pi.rayon

    def motorsfeedback_callback(self, motors_feedback : MotorsFeedback):
        """
        self.left_rear_odometry  = motors_feedback.left_rear_odometry 
        self.right_rear_odometry = motors_feedback.right_rear_odometry
        """
        # RPM variables
        self.left_rear_RPM  = motors_feedback.left_rear_speed  
        self.right_rear_RPM   = motors_feedback.right_rear_speed  

        # Speed variables
        self.left_speed = (self.left_rear_RPM * self.circonference(0.95) * 0.06)
        self.right_speed = (self.right_rear_RPM * self.circonference(0.95) * 0.06)
        self.speed = (self.left_speed+self.right_speed)/2

        self.show_speed()

    def generaldata_callback(self, general_data : GeneralData):
        # battery, temperature, pressure
        self.battery_level  = general_data.battery_level  
        self.temperature  = general_data.temperautre
        self.pressure  = general_data.pressure

        self.show_speed()
    """"
    def motors_order_callback(self, motors_order : MotorsOrder):

        self.motor_right_rear_pwm = motors_order.right_rear_pwm
        self.motor_left_rear_pwm = motors_order.left_rear_pwm
        self.motor_steering_angle = motors_order.steering_angle
    """

    def show_speed(self):

        """"
        if self.motor_right_rear_pwm > STOP or self.motor_left_rear_pwm > STOP:

            if self.ultra_front_left < 20 or self.ultra_front_right < 20 or self.ultra_front_center < 20:

                self.motor_right_rear_pwm = STOP
                self.motor_left_rear_pwm = STOP
                self.get_logger().info("Detecting obstacle <20 cm: Stopping car")

            elif ((20 < self.ultra_front_left < 100) or
                  (20 < self.ultra_front_right < 100) or
                  (20 < self.ultra_front_center < 100)):

                self.motor_right_rear_pwm = min(self.motor_right_rear_pwm, FIRST_MAX)
                self.motor_left_rear_pwm = min(self.motor_left_rear_pwm, FIRST_MAX)
                self.get_logger().info("Detecting obstacle: Speed limit 30%")
        

        msg = MotorsOrder()
        msg.right_rear_pwm = self.motor_right_rear_pwm
        msg.left_rear_pwm = self.motor_left_rear_pwm
        msg.steering_angle = self.motor_steering_angle
        """

        self.publisher_motors_order.publish(msg)




def main(args=None):
    rclpy.init(args=args)

    hmi_node = hmi_node()

    rclpy.spin(hmi_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hmi_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()