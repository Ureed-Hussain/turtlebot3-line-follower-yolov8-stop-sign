#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import torch
import torchvision.transforms as transforms
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import numpy as np


class DepthEstimationNode(Node):
    def __init__(self):
        super().__init__('depth_estimation')

        self.bridge = CvBridge()

        # Load MiDaS model
        self.get_logger().info("🔄 Loading MiDaS model...")
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model_type = "MiDaS_small"
        self.midas = torch.hub.load("intel-isl/MiDaS", self.model_type)
        self.midas.to(self.device).eval()
        self.transform = torch.hub.load("intel-isl/MiDaS", "transforms").dpt_transform

        # Subscribe to camera
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Publisher to /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Threshold distance in "relative" depth map (0.0 = far, 1.0 = close)
        self.depth_threshold = 0.3  # Adjust as needed

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Preprocess
        input_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_tensor = self.transform(input_image).to(self.device)

        # Inference
        with torch.no_grad():
            prediction = self.midas(input_tensor.unsqueeze(0))
            depth_map = prediction.squeeze().cpu().numpy()

        # Normalize depth to [0, 1]
        depth_normalized = cv2.normalize(depth_map, None, 0, 1, cv2.NORM_MINMAX)

        # Focus on center area
        h, w = depth_normalized.shape
        center_crop = depth_normalized[h//3:h*2//3, w//3:w*2//3]
        min_depth = center_crop.min()

        self.get_logger().info(f"🚗 Min center depth: {min_depth:.2f}")

        if min_depth < self.depth_threshold:
            self.stop_robot()
        # else: let the default controller run

    def stop_robot(self):
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_pub.publish(stop_cmd)
        self.get_logger().warn("🛑 Object too close! Stopping robot.")


def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
