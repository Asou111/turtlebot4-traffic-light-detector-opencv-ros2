import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/robot2/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )
        self.vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.speed_green = 5.0
        self.speed_yellow = 0.5
        self.speed_red = 0.0
        self.speed = self.speed_green
        self.deceleration_rate = 0.5
        self.distance_to_travel = 10.0
        self.distance_traveled = 0.0
        self.last_time = self.get_clock().now()
        self.is_red_detected = False
        self.create_timer(0.1, self.update_speed)

    def detect_traffic_light(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_lower1, red_upper1 = np.array([0, 120, 70]), np.array([10, 255, 255])
        red_lower2, red_upper2 = np.array([170, 120, 70]), np.array([180, 255, 255])
        yellow_lower, yellow_upper = np.array([15, 100, 100]), np.array([35, 255, 255])
        green_lower, green_upper = np.array([40, 100, 100]), np.array([90, 255, 255])
        red_mask = cv2.inRange(hsv, red_lower1, red_upper1) + cv2.inRange(hsv, red_lower2, red_upper2)
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)
        if cv2.countNonZero(red_mask) > 500:
            return "ROUGE"
        elif cv2.countNonZero(yellow_mask) > 500:
            return "JAUNE"
        elif cv2.countNonZero(green_mask) > 500:
            return "VERT"
        return "AUCUN"

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        light_color = self.detect_traffic_light(frame)
        if light_color == "ROUGE":
            self.is_red_detected = True
            self.get_logger().info("Feu rouge détecté")
        elif light_color == "JAUNE":
            self.speed = self.speed_yellow
            self.is_red_detected = False
            self.get_logger().info("Feu jaune détecté")
        elif light_color == "VERT":
            self.speed = self.speed_green
            self.is_red_detected = False
            self.get_logger().info("Feu vert détecté")
        else:
            self.speed = self.speed_green
            self.is_red_detected = False
            self.get_logger().info("Aucun feu détecté")

    def update_speed(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if self.is_red_detected:
            if self.speed > self.speed_red:
                self.speed -= self.deceleration_rate * dt
                self.speed = max(self.speed, self.speed_red)
            else:
                self.speed = self.speed_red
        self.distance_traveled += self.speed * dt
        if self.distance_traveled >= self.distance_to_travel:
            self.speed = 0.0
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = 0.0
        self.vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
