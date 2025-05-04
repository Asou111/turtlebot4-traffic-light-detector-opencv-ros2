import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, LaserScan

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10) 
        self.laser_subscription = self.create_subscription(LaserScan, 'robot2/scan', self.laser_callback, 10)  

        self.speed = 2.0 
        self.min_distance = float('inf') 

        self.timer = self.create_timer(0.1, self.timer_callback)  

        self.get_logger().info("Bdina : L3 => Left, Right; R2 => Accelerate. D-Pad Up/Down to adjust speed.")
        
        self.start_moving()

    def start_moving(self):
        twist = Twist()
        twist.linear.x = self.speed
        self.vel_publisher.publish(twist)
        self.get_logger().info(f"Démarrage automatique : vitesse initiale {self.speed} m/s")

    def joy_callback(self, msg):
        if msg.axes[7] == 1.0:  
            self.speed = min(self.speed + 0.2, 2.0)  
        elif msg.axes[7] == -1.0: 
            self.speed = max(0.2, self.speed - 0.2)  

        self.send_velocity_command()

    def laser_callback(self, msg):
        valid_ranges = [d for d in msg.ranges if d > 0.5 and d < float('inf')]
        self.min_distance = min(valid_ranges) if valid_ranges else float('inf')

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = self.speed

        if self.min_distance < 0.6:  
            twist.linear.x = 0.0  
            self.get_logger().warn("Obstacle détecté ! Arrêt du robot.")

        self.vel_publisher.publish(twist)
        self.get_logger().info(f"Vitesse actuelle : {self.speed:.2f} m/s")

    def send_velocity_command(self):
        twist = Twist()
        twist.linear.x = self.speed

        if self.min_distance < 0.6:  
            twist.linear.x = 0.0  
            self.get_logger().warn("Obstacle détecté")

        self.vel_publisher.publish(twist)

def main():
    rclpy.init()
    node = RobotController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
