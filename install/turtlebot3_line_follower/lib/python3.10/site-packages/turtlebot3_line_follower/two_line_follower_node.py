#!/usr/bin/env python3
"""
Control two TurtleBot3 robots (robot1 and robot2) from one process.
Each robot must have:  /<robot_ns>/camera/image_raw  and  /<robot_ns>/cmd_vel
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2


class LineFollower(Node):
    """Follow a white line seen at the bottom of the camera image."""

    def __init__(self, robot_ns: str):
        super().__init__(f"{robot_ns}_line_follower")
        self.ns = robot_ns
        self.bridge = CvBridge()

        # Subscribe to that robot's camera
        self.create_subscription(
            Image,
            f"/{robot_ns}/camera/image_raw",
            self.image_cb,
            10)

        # Publish velocity for that robot
        self.cmd_pub = self.create_publisher(
            Twist,
            f"/{robot_ns}/cmd_vel",
            40)

    # ‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑
    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().error(f"CV‑Bridge error: {exc}")
            return

        h, w, _ = frame.shape
        roi = frame[int(0.75 * h):, :]              # bottom quarter
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # threshold for a WHITE line
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        M = cv2.moments(binary)
        twist = Twist()

        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            error = cx - w // 2
            twist.linear.x = 0.10                   # fwd speed
            twist.angular.z = -float(error) / 100.0 # steer
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.25                  # search turn

        self.cmd_pub.publish(twist)
    # ‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑‑


def main(args=None):
    rclpy.init(args=args)

    # one follower per robot namespace
    robot1 = LineFollower("robot1")
    robot2 = LineFollower("robot2")

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(robot1)
    executor.add_node(robot2)

    try:
        executor.spin()
    finally:
        robot1.destroy_node()
        robot2.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
