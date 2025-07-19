#!/usr/bin/env python3
"""
Robust depth‑based stop detector for robot1 & robot2.
Improvements:
  • larger ROI (lower half of image)
  • uses nearest 5 % pixels, median distance
  • needs N_CONSEC frames before asserting stop
Parameters (ROS):
    safe_meters   – threshold distance to stop
    depth_scale   – relative→metre scale factor
"""

import rclpy, cv2, numpy as np, torch
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge


DEFAULT_SAFE  = 0.8   # m
DEFAULT_SCALE = 2.5   # m when rel_depth == 1
N_CONSEC      = 3     # consecutive frames to trigger


# ─── MiDaS wrapper ───────────────────────────────────────────────────────────
class MidasDepth:
    def __init__(self, node_logger):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        node_logger.info(f"Loading MiDaS_small on {self.device} …")
        self.net = torch.hub.load("intel-isl/MiDaS", "MiDaS_small").to(self.device).eval()
        tf_mod    = torch.hub.load("intel-isl/MiDaS", "transforms")
        self.tf   = tf_mod.small_transform
        node_logger.info("MiDaS loaded ✓")

    def __call__(self, bgr):
        rgb  = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        inp  = self.tf(rgb).to(self.device)
        with torch.no_grad():
            pred = self.net(inp)
        pred  = torch.nn.functional.interpolate(
                    pred.unsqueeze(1), size=rgb.shape[:2],
                    mode="bicubic", align_corners=False
                ).squeeze().cpu().numpy()
        pred  = (pred - pred.min()) / (pred.max() - pred.min() + 1e-6)
        return 1.0 - pred          # 1 = near


# ─── Watcher for a single robot ──────────────────────────────────────────────
class DepthWatcher(Node):
    def __init__(self, ns: str, depth_fn: MidasDepth):
        super().__init__(f"{ns}_depth")
        self.ns       = ns
        self.depth_fn = depth_fn
        self.bridge   = CvBridge()

        # parameters
        self.declare_parameter("safe_meters",  DEFAULT_SAFE)
        self.declare_parameter("depth_scale",  DEFAULT_SCALE)

        # pubs
        self.depth_pub = self.create_publisher(Image, f"/{ns}/depth_image", 1)
        self.stop_pub  = self.create_publisher(Bool,  f"/{ns}/depth_stop",  1)
        # seed topics so `ros2 topic list` shows them
        self.depth_pub.publish(Image())
        self.stop_pub.publish(Bool(data=False))

        # sub
        self.create_subscription(Image,
                                 f"/{ns}/camera/image_raw",
                                 self.cb, 6)

        self.consec = 0
        self.get_logger().info("DepthWatcher ready")

    # ─────────────────────────────────────────────────────────────────────────
    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        depth = self.depth_fn(frame)

        # RViz mono8 image
        d_vis = (depth * 255).astype(np.uint8)
        d_msg = self.bridge.cv2_to_imgmsg(d_vis, encoding="mono8")
        d_msg.header = msg.header
        self.depth_pub.publish(d_msg)

        # ROI: lower half
        h, w = depth.shape
        roi  = depth[h//2:, :]                       # (h/2…h, full width)
        flat = roi.flatten()
        closest = np.partition(flat, int(0.05*flat.size))[: int(0.05*flat.size)]
        rel_med = float(np.median(closest))

        scale = self.get_parameter("depth_scale").value
        safe  = self.get_parameter("safe_meters").value
        est_m = scale * rel_med

        blocked = est_m < safe
        self.consec = self.consec + 1 if blocked else 0
        stopped = self.consec >= N_CONSEC
        self.stop_pub.publish(Bool(data=stopped))

        self.get_logger().debug(
            f"{self.ns}: est={est_m:.2f} m  blocked={blocked} → stop={stopped}"
        )


# ─── main ────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    tmp_logger = rclpy.logging.get_logger("depth_loader")
    depth_fn   = MidasDepth(tmp_logger)

    nodes = [DepthWatcher("robot1", depth_fn),
             DepthWatcher("robot2", depth_fn)]

    exec_ = MultiThreadedExecutor()
    for n in nodes: exec_.add_node(n)

    try:
        exec_.spin()
    finally:
        exec_.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
