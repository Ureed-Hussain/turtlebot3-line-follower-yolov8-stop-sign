#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarStop(Node):
    def __init__(self):
        super().__init__('lidar_stop')
        
        self.stop_distance = 0.5  # Stop if something is closer than 0.5 m
        self.obstacle_detected = False

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',  # default TurtleBot3 lidar topic
            self.scan_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',  # this will override the line follower if needed
            10
        )

        self.timer = self.create_timer(0.1, self.check_and_publish)

    def scan_callback(self, msg):
        # Only consider front 40 degrees (20 left + 20 right)
        num_ranges = len(msg.ranges)
        center_idx = num_ranges // 2
        window = num_ranges // 18  # ~20 degrees
        front_ranges = msg.ranges[center_idx - window: center_idx + window]

        # Filter out 0.0 or inf values
        front_ranges = [r for r in front_ranges if 0.05 < r < msg.range_max]

        if front_ranges and min(front_ranges) < self.stop_distance:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def check_and_publish(self):
        if self.obstacle_detected:
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_pub.publish(stop_cmd)
            self.get_logger().info("🚨 Obstacle detected. Stopping.")
        # Else: do nothing — line follower keeps sending /cmd_vel

def main(args=None):
    rclpy.init(args=args)
    node = LidarStop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
