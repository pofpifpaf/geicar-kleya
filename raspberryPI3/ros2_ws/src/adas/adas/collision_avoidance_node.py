import rclpy
from rclpy.node import Node

from interfaces.msg import Ultrasonic, MotorsOrder


# les seuils 

STOP = 50
FRONT_FIRST_MAX = 65
FRONT_SECOND_MAX = 80
REAR_MAX = 40
SAFE_DISTANCE_FRONT_STOP = 20
SAFE_DISTANCE_FRONT_SLOW = 50
SAFE_DISTANCE_FRONT_MODERATE = 100
SAFE_DISTANCE_REAR_STOP = 20
SAFE_DISTANCE_REAR_SLOW = 100
#FIRST_MAX = 65


class collision_avoidance(Node):

    def __init__(self):

        super().__init__('collision_avoidance_node')        
        self.publisher_motors_order = self.create_publisher(MotorsOrder, 'motors_order', 10)

        self.subscription = self.create_subscription(Ultrasonic,'us_data', self.ultrasonic_callback, 10)
        self.subscription = self.create_subscription(MotorsOrder,'motors_order_raw', self.motors_order_callback, 10)
        self.subscription  # prevent unused variable warning
         

        # valeurs des 6 ultrasons () 
        self.ultra_front_left = 300
        self.ultra_front_right = 300
        self.ultra_front_center = 300
        self.ultra_rear_left = 300
        self.ultra_rear_right = 300
        self.ultra_rear_center = 300

        # valeurs moteurs 
        self.motor_right_rear_pwm = 50
        self.motor_left_rear_pwm = 50
        self.motor_steering_angle = 0

        self.get_logger().info("collision_avoidance_node READY")


    def ultrasonic_callback(self, us_data : Ultrasonic):
        # mis a jour des valeurs 
        self.ultra_front_left = us_data.front_left
        self.ultra_front_right = us_data.front_right
        self.ultra_front_center = us_data.front_center
        self.ultra_rear_left = us_data.rear_left
        self.ultra_rear_right = us_data.rear_right
        self.ultra_rear_center = us_data.rear_center


        self.detect_collision()

    def motors_order_callback(self, motors_order : MotorsOrder):

        self.motor_right_rear_pwm = motors_order.right_rear_pwm
        self.motor_left_rear_pwm = motors_order.left_rear_pwm
        self.motor_steering_angle = motors_order.steering_angle

    def detect_collision(self):
        # Détection avant 
        if self.motor_right_rear_pwm > STOP or self.motor_left_rear_pwm > STOP:
            min_front = min(self.ultra_front_left, self.ultra_front_center, self.ultra_front_right)

            if min_front < SAFE_DISTANCE_FRONT_STOP:
            #if self.ultra_front_left < 20 or self.ultra_front_right < 20 or self.ultra_front_center < 20:
                self.motor_right_rear_pwm = STOP
                self.motor_left_rear_pwm = STOP
                self.get_logger().info("Obstacle avant < 20 cm : Arret immédiat")

             #elif ((20 < self.ultra_front_left < 100) or
                   #(20 < self.ultra_front_right < 100) or
                   #(20 < self.ultra_front_center < 100)):
            elif min_front < SAFE_DISTANCE_FRONT_SLOW:       

                self.motor_right_rear_pwm = min(self.motor_right_rear_pwm, FRONT_FIRST_MAX)
                self.motor_left_rear_pwm = min(self.motor_left_rear_pwm, FRONT_FIRST_MAX)
                self.get_logger().info("Obstacle d'avant qui se trouve a une distance entre 20-50cm : Speed limit 30%")
        # Détection arrière
        min_rear = min(self.ultra_rear_left, self.ultra_rear_center, self.ultra_rear_right)
        if self.motor_right_rear_pwm < STOP and self.motor_left_rear_pwm < STOP:  # marche arrière
            if min_rear < SAFE_DISTANCE_REAR_STOP:
                self.motor_right_rear_pwm = STOP
                self.motor_left_rear_pwm = STOP
                self.get_logger().info("Obstacle arrière < 20 cm : STOP immédiat")
         elif min_rear < SAFE_DISTANCE_REAR_SLOW:
                reduced_pwm = min(self.motor_right_rear_pwm, REAR_MAX)
                self.motor_right_rear_pwm = reduced_pwm
                self.motor_left_rear_pwm = reduced_pwm
                self.get_logger().info("Obstacle arrière 20-100 cm : ralentissement")
        
        
        # Empêcher accélération en virage si obstacle proche sur un côté
        if self.motor_steering_angle > 10 and min_front < 50:  # tourne à droite
            self.motor_right_rear_pwm = min(self.motor_right_rear_pwm, STOP)
         elif self.motor_steering_angle < -10 and min_front < 50:  # tourne à gauche
            self.motor_left_rear_pwm = min(self.motor_left_rear_pwm, STOP)
       
       
        # Publier ordre moteur
        msg = MotorsOrder()
        msg.right_rear_pwm = self.motor_right_rear_pwm
        msg.left_rear_pwm = self.motor_left_rear_pwm
        msg.steering_angle = self.motor_steering_angle

        self.publisher_motors_order.publish(msg)




def main(args=None):
    rclpy.init(args=args)

    collision_avoidance_node = collision_avoidance()

    rclpy.spin(collision_avoidance_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    collision_avoidance_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()