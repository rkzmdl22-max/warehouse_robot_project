#!/usr/bin/env python3
"""
Bottle Detector Node — GATE 2-B Real RGB-D Vision Implementation
================================================================
Subscribes to TIAGo head_front_camera RGB + Depth images,
performs OpenCV HSV-based colour segmentation to find bottle candidates,
projects the centroid pixel to 3D using depth + camera intrinsics,
and publishes:
  /detected_bottle_pose       (geometry_msgs/PoseStamped)
  /detected_bottle_stopped    (std_msgs/Bool)
  /bottle_detector/log        (std_msgs/String)

No external AI framework (YOLO etc.) is used.
Camera frame: head_front_camera_optical_frame  (orbbec_astra_pro convention)

Topic names are intentionally kept identical to the original stub so that
SideGraspVerifier and TRON1MissionFSM require zero modification.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, String

from cv_bridge import CvBridge
import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Camera intrinsics fallback
# (overridden at runtime when /station_rgbd/rgb/camera_info arrives)
# Values match orbbec_astra_pro 640x480 @ 58° VFOV typical parameters.
# ---------------------------------------------------------------------------
_DEFAULT_FX = 525.0
_DEFAULT_FY = 525.0
_DEFAULT_CX = 319.5
_DEFAULT_CY = 239.5


# ---------------------------------------------------------------------------
# HSV colour ranges for bottle detection
# Tuned for the orange/brown cardboard box_1 in warehouse.world AND
# for a generic "bottle" hue range (green/amber bottles are common).
# Two ranges are tried in order; the first match is used.
# ---------------------------------------------------------------------------
# Range A — orange / brown (matches box_1 ambient colour 0.8 0.6 0.2)
HSV_LOW_A  = np.array([  8,  80,  80], dtype=np.uint8)
HSV_HIGH_A = np.array([ 30, 255, 255], dtype=np.uint8)

# Range B — green bottles (fallback)
HSV_LOW_B  = np.array([ 35,  60,  60], dtype=np.uint8)
HSV_HIGH_B = np.array([ 85, 255, 200], dtype=np.uint8)

# Range C — blue bottle carried on TRON1 tray
# TRON1 carry_bottle_link material: rgba 0.2 0.5 1.0 1.0
HSV_LOW_C  = np.array([ 90, 70, 50], dtype=np.uint8)
HSV_HIGH_C = np.array([135, 255, 255], dtype=np.uint8)

TRON1_MIN_CONTOUR_AREA = 100


# Minimum contour area (pixels²) to accept as a detection
MIN_CONTOUR_AREA = 400

# Depth sampling window half-size (pixels) around centroid
DEPTH_WINDOW = 5

# Velocity threshold for "stopped" judgement (m/s)
VELOCITY_THRESHOLD = 0.01

# Camera optical frame — orbbec-astra-pro Gazebo Classic convention
# (libgazebo_ros_openni_kinect.so publishes with frameName=${name}_optical_frame)
CAMERA_FRAME = 'station_camera_color_optical_frame'

# QoS for sensor topics
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class BottleDetectorNode(Node):
    """
    Real RGB-D bottle detector.

    Pipeline:
      RGB callback  → HSV segmentation → centroid pixel (u, v)
      Depth callback → depth at (u, v)
      → 3D projection using camera intrinsics
      → publish /detected_bottle_pose
      → velocity check → publish /detected_bottle_stopped
    """

    def __init__(self):
        super().__init__('bottle_detector_node')

        # --- Publishers (topic names unchanged from original stub) ---
        self.pose_pub   = self.create_publisher(PoseStamped, '/detected_bottle_pose',    10)
        self.status_pub = self.create_publisher(Bool,        '/detected_bottle_stopped', 10)
        self.log_pub    = self.create_publisher(String,      '/bottle_detector/log',     10)
        self.tron1_cargo_pub = self.create_publisher(
            Bool, '/tron1/tray_bottle_detected', 10
        )


        # --- Subscribers ---
        self.create_subscription(
            Image, '/station_rgbd/rgb/image_raw',
            self._rgb_callback, SENSOR_QOS,
        )
        self.create_subscription(
            Image, '/station_rgbd/depth/image_raw',
            self._depth_callback, SENSOR_QOS,
        )
        self.create_subscription(
            CameraInfo, '/station_rgbd/rgb/camera_info',
            self._camera_info_callback, 10,
        )

        self.create_subscription(
            Image, '/tron1/camera/rgb/image_raw',
            self._tron1_rgb_callback, SENSOR_QOS,
        )

        # --- State ---
        self.bridge = CvBridge()

        self.latest_rgb   = None   # np.ndarray BGR uint8
        self.latest_depth = None   # np.ndarray float32 (metres)

        # Camera intrinsics (updated from camera_info topic)
        self.fx = _DEFAULT_FX
        self.fy = _DEFAULT_FY
        self.cx = _DEFAULT_CX
        self.cy = _DEFAULT_CY
        self.intrinsics_received = False

        # 3-D position history for velocity estimation
        self.prev_pos   = None   # [x, y, z]
        self.prev_stamp = None   # rclpy.time.Time

        self.is_stopped = False

        # Counters for debug reporting
        self._rgb_recv   = 0
        self._depth_recv = 0

        self.get_logger().info(
            'BottleDetectorNode (GATE 2-B) initialised. '
            'Waiting for /station_rgbd/rgb/image_raw and '
            '/station_rgbd/depth/image_raw …'
        )

        # Processing timer — 10 Hz
        self.create_timer(0.1, self._perception_loop)

    # -----------------------------------------------------------------------
    # Subscriber callbacks
    # -----------------------------------------------------------------------

    def _rgb_callback(self, msg: Image):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._rgb_recv += 1
            if self._rgb_recv == 1:
                self.get_logger().info('First RGB frame received.')
        except Exception as exc:
            self.get_logger().warn(f'RGB conversion failed: {exc}')

    def _tron1_rgb_callback(self, msg: Image):
        """G14: Detect blue bottle carried on the TRON1 tray."""
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().warning(
                f'TRON1 RGB conversion failed: {exc}'
            )
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(
            hsv,
            HSV_LOW_C,
            HSV_HIGH_C
        )

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, kernel
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, kernel
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        max_area = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area

        detected = max_area >= TRON1_MIN_CONTOUR_AREA

        cargo_msg = Bool()
        cargo_msg.data = detected
        self.tron1_cargo_pub.publish(cargo_msg)

        if detected:
            self.get_logger().info(
                f'G14 PASS: TRON1 tray/bottle detected '
                f'(blue area={max_area:.0f}px)'
            )

    def _depth_callback(self, msg: Image):
        try:
            # 32FC1 → float32 numpy array in metres
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self._depth_recv += 1
            if self._depth_recv == 1:
                self.get_logger().info('First Depth frame received.')
        except Exception as exc:
            self.get_logger().warn(f'Depth conversion failed: {exc}')

    def _camera_info_callback(self, msg: CameraInfo):
        if not self.intrinsics_received:
            # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.intrinsics_received = True
            self.get_logger().info(
                f'Camera intrinsics received: '
                f'fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}'
            )

    # -----------------------------------------------------------------------
    # Main perception loop
    # -----------------------------------------------------------------------

    def _perception_loop(self):
        # Guard: both frames must be available
        if self.latest_rgb is None:
            self.get_logger().debug(
                'Waiting for RGB frame … '
                f'(rgb={self._rgb_recv}, depth={self._depth_recv})',
                throttle_duration_sec=5.0,
            )
            return
        if self.latest_depth is None:
            self.get_logger().debug(
                'Waiting for Depth frame … '
                f'(rgb={self._rgb_recv}, depth={self._depth_recv})',
                throttle_duration_sec=5.0,
            )
            return

        rgb   = self.latest_rgb.copy()
        depth = self.latest_depth.copy()

        # --- Step 1: HSV segmentation ---
        centroid = self._detect_bottle_centroid(rgb)

        if centroid is None:
            self._publish_log('Detection FAILED — no bottle candidate found in HSV mask.')
            # Publish stopped=False so FSM does not proceed on stale data
            self.status_pub.publish(Bool(data=False))
            return

        u, v = centroid
        self.get_logger().debug(f'Bottle candidate centroid pixel: u={u}, v={v}')

        # --- Step 2: Depth sampling ---
        z_metres = self._sample_depth(depth, u, v)

        if z_metres is None or z_metres <= 0.0 or not math.isfinite(z_metres):
            self._publish_log(
                f'Detection FAILED — invalid depth at ({u},{v}): z={z_metres}'
            )
            self.status_pub.publish(Bool(data=False))
            return

        # --- Step 3: Pixel → 3D projection ---
        x3d = (u - self.cx) * z_metres / self.fx
        y3d = (v - self.cy) * z_metres / self.fy
        z3d = z_metres

        self.get_logger().debug(
            f'3D position: x={x3d:.3f} y={y3d:.3f} z={z3d:.3f} m  '
            f'depth_raw={z_metres:.3f} m'
        )

        # --- Step 4: Velocity → stopped check ---
        now = self.get_clock().now()
        if self.prev_pos is not None and self.prev_stamp is not None:
            dt = (now - self.prev_stamp).nanoseconds * 1e-9
            if dt > 0.0:
                dx = x3d - self.prev_pos[0]
                dy = y3d - self.prev_pos[1]
                dz = z3d - self.prev_pos[2]
                velocity = math.sqrt(dx*dx + dy*dy + dz*dz) / dt
                self.is_stopped = (velocity < VELOCITY_THRESHOLD)
            else:
                self.is_stopped = False
        else:
            self.is_stopped = False

        self.prev_pos   = [x3d, y3d, z3d]
        self.prev_stamp = now

        # --- Step 5: Publish ---
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = now.to_msg()
        pose_msg.header.frame_id = CAMERA_FRAME
        pose_msg.pose.position.x = x3d
        pose_msg.pose.position.y = y3d
        pose_msg.pose.position.z = z3d
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

        stop_msg = Bool()
        stop_msg.data = self.is_stopped
        self.status_pub.publish(stop_msg)

        self._publish_log(
            f'DETECTED x={x3d:.3f} y={y3d:.3f} z={z3d:.3f} '
            f'stopped={self.is_stopped}'
        )

    # -----------------------------------------------------------------------
    # Helper: HSV segmentation → centroid pixel
    # -----------------------------------------------------------------------

    def _detect_bottle_centroid(self, bgr: np.ndarray):
        """
        Returns (u, v) centroid pixel of the largest colour blob,
        or None if no candidate found.

        Tries HSV range A (orange/brown) first, then range B (green).
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        best_area    = 0
        best_centroid = None

        for low, high in [(HSV_LOW_A, HSV_HIGH_A), (HSV_LOW_B, HSV_HIGH_B)]:
            mask = cv2.inRange(hsv, low, high)

            # Morphological clean-up: remove noise, fill holes
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area    = cv2.contourArea(largest)

            if area < MIN_CONTOUR_AREA:
                continue

            M = cv2.moments(largest)
            if M['m00'] == 0:
                continue

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            if area > best_area:
                best_area     = area
                best_centroid = (cx, cy)

        return best_centroid

    # -----------------------------------------------------------------------
    # Helper: robust depth sampling around centroid
    # -----------------------------------------------------------------------

    def _sample_depth(self, depth: np.ndarray, u: int, v: int) -> float:
        """
        Returns median of finite, positive depth values in a window
        centred on (u, v).  Returns None if no valid pixels found.
        """
        h, w = depth.shape[:2]
        u0 = max(0, u - DEPTH_WINDOW)
        u1 = min(w, u + DEPTH_WINDOW + 1)
        v0 = max(0, v - DEPTH_WINDOW)
        v1 = min(h, v + DEPTH_WINDOW + 1)

        patch = depth[v0:v1, u0:u1]
        valid = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 10.0)]

        if valid.size == 0:
            return None

        return float(np.median(valid))

    # -----------------------------------------------------------------------
    # Helper: publish to /bottle_detector/log
    # -----------------------------------------------------------------------

    def _publish_log(self, text: str):
        msg = String()
        msg.data = text
        self.log_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = BottleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
