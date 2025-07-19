#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        height, width, _ = frame.shape
        crop = frame[int(height*0.7):height, 0:width]

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

        # For WHITE LINE:
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        M = cv2.moments(binary)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            error = cx - width // 2

            twist = Twist()
            twist.linear.x = 0.1
            twist.angular.z = -float(error) / 100.0
            self.cmd_pub.publish(twist)
        else:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.2
            self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()