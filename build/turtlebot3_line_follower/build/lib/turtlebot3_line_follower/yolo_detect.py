#!/usr/bin/env python3
import rclpy, cv2, time
from rclpy.node        import Node
from sensor_msgs.msg    import Image
from geometry_msgs.msg  import Twist
from cv_bridge          import CvBridge
from ultralytics        import YOLO
import numpy as np

# ───────────── parameters you may tweak ─────────────
STOP_CLASS_ID  = 11                # COCO stop‑sign id
STOP_SECS      = 2.0               # stand still
PAUSE_SECS     = 1.5               # pause before rotating
ROT_SPEED      = 0.5               # rad/s
MAX_ROT_SECS   = 7.0               # fail‑safe spin timeout
NEEDED_FRAMES  = 5                 # consecutive “line seen” frames
FWD_SPEED      = 0.10
ANG_GAIN       = 0.01              # sharper turns
YOLO_EVERY_N   = 5                 # run YOLO every N images
# ────────────────────────────────────────────────────


class StopSignFollower(Node):
    def __init__(self):
        super().__init__('stop_sign_line_follower')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.img_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.model = YOLO('yolov8n.pt')
        self.model.fuse()

        # state machine
        self.state       = 'FOLLOW'
        self.state_until = time.time()
        self.spin_start  = 0.0
        self.line_hits   = 0
        self.frame_cnt   = 0

        self.get_logger().info('🚗 Line‑follower + precise STOP sign logic ready.')

    # ───────────────────────────── helper
    def set_state(self, new, dur=0.0):
        self.state = new
        self.state_until = time.time() + dur
        self.get_logger().info(f'→ STATE = {new}')

    def publish(self, lin, ang):
        msg = Twist(); msg.linear.x = lin; msg.angular.z = ang
        self.cmd_pub.publish(msg)

    # ───────────────────────────── image callback
    def img_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = frame.shape
        now = time.time()

        # ── ROTATE state ─────────────────────────────────────────────
        if self.state == 'ROTATE':
            # fail‑safe timeout
            if now - self.spin_start > MAX_ROT_SECS:
                self.get_logger().warn('⚠️  Spin timeout – giving up, resume FOLLOW.')
                self.set_state('FOLLOW')
                return

            # detect white line in *full* frame (adaptive ROI)
            gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, bin_full = cv2.threshold(gray_full, 200, 255, cv2.THRESH_BINARY)
            if cv2.countNonZero(bin_full) > 8000:          # line seen
                self.line_hits += 1
            else:
                self.line_hits = 0

            if self.line_hits >= NEEDED_FRAMES:
                self.get_logger().info('✅ Line reacquired – stop spin.')
                self.publish(0.0, 0.0)
                self.set_state('FOLLOW')
                return

            # keep spinning
            self.publish(0.0, ROT_SPEED)
            return

        # ── PAUSE state ──────────────────────────────────────────────
        if self.state in ('STOP', 'PAUSE'):
            if now >= self.state_until:
                if self.state == 'STOP':
                    self.set_state('PAUSE', PAUSE_SECS)
                    self.publish(0.0, 0.0)
                else:  # finished PAUSE → start spin
                    self.spin_start = now
                    self.line_hits  = 0
                    self.set_state('ROTATE')
            else:
                self.publish(0.0, 0.0)
            return

        # ── FOLLOW state  (normal line following) ────────────────────
        crop = frame[int(h*0.7):, :]
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        M = cv2.moments(binary)
        if M['m00'] > 0:
            cx   = int(M['m10']/M['m00'])
            err  = cx - w//2
            self.publish(FWD_SPEED, -ANG_GAIN * err)
        else:
            self.publish(0.0, 0.25)  # slow search

        # ── YOLO every N frames ─────────────────────────────────────
        self.frame_cnt += 1
        if self.frame_cnt % YOLO_EVERY_N:
            return

        res = self.model.predict(frame, imgsz=416, conf=0.4, verbose=False)[0]
        for c, box in zip(res.boxes.cls, res.boxes.xyxy):
            if int(c.item()) != STOP_CLASS_ID:
                continue
            x1, y1, x2, y2 = map(int, box.tolist())
            area  = (x2-x1)*(y2-y1)
            cxbox = (x1+x2)//2
            if 0.3*w < cxbox < 0.7*w and area > 0.02*w*h:
                self.get_logger().info('🛑 STOP sign detected!')
                self.publish(0.0, 0.0)
                self.set_state('STOP', STOP_SECS)
                break


def main():
    rclpy.init()
    node = StopSignFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
