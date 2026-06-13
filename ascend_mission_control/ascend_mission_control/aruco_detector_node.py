#!/usr/bin/env python3
"""
ascend_vision/aruco_detector_node.py  — FIXED VERSION

Root causes of the segfault (now resolved):

  BUG 1 — Wrong grayscale conversion:
    OLD (broken):  gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                   ↑ frame is BGR (from cv_bridge desired_encoding='bgr8')
                     but COLOR_RGB2GRAY tells OpenCV it's RGB.
                     If the image happens to arrive as mono8, frame.shape
                     has no channel dim → cvtColor crashes with "bad channel count"
    FIX:           desired_encoding='bgr8' forces BGR always
                   gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

  BUG 2 — Wrong detector API selection:
    OLD (broken):  aruco.detectMarkers(gray, self._aruco_dict, parameters=self._aruco_params)
                   On OpenCV 4.6 the C++ DetectorParameters struct layout changed between
                   patch versions. Passing a Python-constructed DetectorParameters() into
                   detectMarkers() causes a memory layout mismatch → segfault.
                   On OpenCV 4.8+ detectMarkers is soft-deprecated and ArucoDetector exists.
    FIX:           Use ArucoDetector (new API) if available (4.8+),
                   fall back to detectMarkers + DetectorParameters_create() for 4.6.
                   DetectorParameters_create() is the safe 4.6 factory, not the constructor.

  BUG 3 — Redundant double import and leftover debug logs:
    OLD:  __init__ re-imported cv2 and aruco as locals, shadowing module-level names.
    FIX:  Single module-level import only.

Topics published:
  /ascend/vision/aruco_detection       (std_msgs/String  JSON)
  /ascend/vision/aruco_debug_image     (sensor_msgs/Image, optional)

Topics subscribed:
  /camera/image        (sensor_msgs/Image)
  /camera/camera_info  (sensor_msgs/CameraInfo)

JSON schema published on /ascend/vision/aruco_detection:
  {
    "is_found":   bool,
    "angle_x":    float,   # rad, +right of centre
    "angle_y":    float,   # rad, +below centre (= +forward in drone body frame)
    "marker_id":  int,
    "pixel_x":    float,
    "pixel_y":    float,
    "distance_m": float
  }
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import cv2
import cv2.aruco as aruco
import numpy as np
import json
import math

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge

# ── OpenCV version detection ──────────────────────────────────────────────────
_CV_MAJOR, _CV_MINOR = (int(x) for x in cv2.__version__.split(".")[:2])
_USE_NEW_API = hasattr(aruco, "ArucoDetector")   # True on 4.8+

# ── ArUco dictionary map ──────────────────────────────────────────────────────
ARUCO_DICTS = {
    "4X4_50":   aruco.DICT_4X4_50,
    "4X4_100":  aruco.DICT_4X4_100,
    "5X5_100":  aruco.DICT_5X5_100,
    "6X6_250":  aruco.DICT_6X6_250,
    "7X7_1000": aruco.DICT_7X7_1000,
}


def _make_detector_params():
    """
    Create DetectorParameters safely across OpenCV versions.

    OpenCV 4.6: DetectorParameters() constructor sometimes has ABI issues;
                DetectorParameters_create() is the safe factory method.
    OpenCV 4.7+: DetectorParameters() constructor is safe.
    OpenCV 4.8+: Same as 4.7 (new ArucoDetector wraps it).
    """
    if hasattr(aruco, "DetectorParameters_create"):
        return aruco.DetectorParameters_create()
    return aruco.DetectorParameters()


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__("aruco_detector_node")

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter("marker_size_m",     0.3)
        self.declare_parameter("aruco_dict",        "4X4_50")
        self.declare_parameter("target_marker_id",  0)
        self.declare_parameter("publish_debug",     True)
        self.declare_parameter("camera_topic",      "/camera/image")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")

        self._marker_size = self.get_parameter("marker_size_m").value
        dict_name         = self.get_parameter("aruco_dict").value
        self._target_id   = self.get_parameter("target_marker_id").value
        self._debug       = self.get_parameter("publish_debug").value
        cam_topic         = self.get_parameter("camera_topic").value
        info_topic        = self.get_parameter("camera_info_topic").value

        # ── ArUco detector — version-aware ────────────────────────────────
        aruco_dict_id      = ARUCO_DICTS.get(dict_name, aruco.DICT_4X4_50)
        self._aruco_dict   = aruco.getPredefinedDictionary(aruco_dict_id)
        self._aruco_params = _make_detector_params()

        if _USE_NEW_API:
            # OpenCV 4.8+ — use ArucoDetector class
            self._detector = aruco.ArucoDetector(self._aruco_dict, self._aruco_params)
            self.get_logger().info(
                f"OpenCV {cv2.__version__} — using ArucoDetector (new API)"
            )
        else:
            # OpenCV 4.6/4.7 — use module-level detectMarkers
            self._detector = None
            self.get_logger().info(
                f"OpenCV {cv2.__version__} — using aruco.detectMarkers (legacy API)"
            )

        # ── Camera intrinsics ─────────────────────────────────────────────
        self._fx            = None
        self._fy            = None
        self._cx            = None
        self._cy            = None
        self._camera_matrix = None
        self._dist_coeffs   = None

        # ── cv_bridge ─────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── QoS ──────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.sub_image = self.create_subscription(
            Image, cam_topic, self._cb_image, sensor_qos
        )
        self.sub_info = self.create_subscription(
            CameraInfo, info_topic, self._cb_camera_info, sensor_qos
        )

        self.pub_detection = self.create_publisher(
            String, "/ascend/vision/aruco_detection", reliable_qos
        )
        self.pub_debug = (
            self.create_publisher(Image, "/ascend/vision/aruco_debug_image", sensor_qos)
            if self._debug
            else None
        )

        self.get_logger().info(
            f"ArUco detector ready — dict={dict_name} "
            f"target_id={self._target_id} "
            f"marker_size={self._marker_size}m "
            f"camera={cam_topic}"
        )

    # ─────────────────────────────────────────────────────────────────────
    #  Camera info
    # ─────────────────────────────────────────────────────────────────────

    def _cb_camera_info(self, msg: CameraInfo):
        K = msg.k
        self._fx = K[0]
        self._fy = K[4]
        self._cx = K[2]
        self._cy = K[5]
        self._camera_matrix = np.array(
            [[self._fx, 0, self._cx],
             [0, self._fy, self._cy],
             [0, 0, 1]],
            dtype=np.float64,
        )
        self._dist_coeffs = np.array(msg.d, dtype=np.float64)

    # ─────────────────────────────────────────────────────────────────────
    #  Image callback
    # ─────────────────────────────────────────────────────────────────────

    def _cb_image(self, msg: Image):

        # ── Step 1: initialise intrinsics from image size if info not yet received
        if self._fx is None:
            self._fx = self._fy = float(max(msg.width, msg.height))
            self._cx = msg.width  / 2.0
            self._cy = msg.height / 2.0
            self._camera_matrix = np.array(
                [[self._fx, 0, self._cx],
                 [0, self._fy, self._cy],
                 [0, 0, 1]],
                dtype=np.float64,
            )
            self._dist_coeffs = np.zeros(5, dtype=np.float64)
            self.get_logger().warn(
                "camera_info not yet received — using image-centre fallback intrinsics.",
                throttle_duration_sec=5.0,
            )

        # ── Step 2: decode image
        # FIX: always request 'bgr8' so cv_bridge handles ANY source encoding
        # (rgb8, bgr8, rgba8, mono8, etc.) and always gives us a 3-channel BGR frame.
        # This eliminates the "bad channel count" crash when mono8 arrives.
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {e}", throttle_duration_sec=2.0)
            return

        if frame is None or frame.size == 0:
            return

        # ── Step 3: grayscale
        # FIX: frame is always BGR from step 2 → use COLOR_BGR2GRAY (not RGB)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = np.ascontiguousarray(gray)

        # ── Step 4: detect markers — version-aware
        corners, ids = self._detect(gray)

        # ── Step 5: build result
        result = _not_found()

        if ids is not None and len(ids) > 0:
            for i, marker_id in enumerate(ids.flatten()):
                if self._target_id >= 0 and int(marker_id) != self._target_id:
                    continue

                c     = corners[i][0]                   # (4,2)
                cx_px = float(np.mean(c[:, 0]))
                cy_px = float(np.mean(c[:, 1]))

                angle_x = math.atan2(cx_px - self._cx, self._fx)
                angle_y = math.atan2(cy_px - self._cy, self._fy)

                result = {
                    "is_found":   True,
                    "angle_x":    round(angle_x,    5),
                    "angle_y":    round(angle_y,    5),
                    "marker_id":  int(marker_id),
                    "pixel_x":    round(cx_px, 2),
                    "pixel_y":    round(cy_px, 2),
                    "distance_m": round(self._estimate_distance(corners[i]), 3),
                }
                break

            if self._debug:
                aruco.drawDetectedMarkers(frame, corners, ids)

        # ── Step 6: publish JSON
        out = String()
        out.data = json.dumps(result)
        self.pub_detection.publish(out)

        # ── Step 7: debug image
        if self._debug and self.pub_debug is not None:
            cv2.drawMarker(
                frame,
                (int(self._cx), int(self._cy)),
                (0, 255, 0), cv2.MARKER_CROSS, 20, 2,
            )
            if result["is_found"]:
                cv2.arrowedLine(
                    frame,
                    (int(self._cx), int(self._cy)),
                    (int(result["pixel_x"]), int(result["pixel_y"])),
                    (0, 0, 255), 2,
                )
                cv2.putText(
                    frame,
                    f'id={result["marker_id"]} '
                    f'ax={result["angle_x"]:.3f} '
                    f'ay={result["angle_y"]:.3f} '
                    f'd={result["distance_m"]:.2f}m',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2,
                )
            try:
                dbg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                dbg.header = msg.header
                self.pub_debug.publish(dbg)
            except Exception:
                pass

    # ─────────────────────────────────────────────────────────────────────
    #  Detection — version-aware dispatch
    # ─────────────────────────────────────────────────────────────────────

    def _detect(self, gray: np.ndarray):
        """
        Returns (corners, ids) regardless of OpenCV version.
        Uses ArucoDetector.detectMarkers on 4.8+,
        falls back to aruco.detectMarkers on 4.6/4.7.
        """
        if _USE_NEW_API:
            # self._detector is ArucoDetector instance
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            # Legacy: module-level function
            corners, ids, _ = aruco.detectMarkers(
                gray, self._aruco_dict, parameters=self._aruco_params
            )

        self.get_logger().info(
            f"ids={ids}, corners={0 if corners is None else len(corners)}"
        )
        return corners, ids

    # ─────────────────────────────────────────────────────────────────────
    #  Distance estimation via solvePnP
    # ─────────────────────────────────────────────────────────────────────

    def _estimate_distance(self, corners) -> float:
        if self._camera_matrix is None:
            return 0.0

        half = self._marker_size / 2.0
        obj_pts = np.array(
            [[-half, half, 0], [half, half, 0],
             [half, -half, 0], [-half, -half, 0]],
            dtype=np.float64,
        )
        img_pts = corners[0].astype(np.float64)

        try:
            ok, _, tvec = cv2.solvePnP(
                obj_pts, img_pts,
                self._camera_matrix, self._dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if ok:
                return float(np.linalg.norm(tvec))
        except Exception:
            pass

        # Pixel-size fallback
        edge_px = float(np.linalg.norm(corners[0][0] - corners[0][1]))
        if edge_px > 1.0 and self._fx is not None:
            return (self._marker_size * self._fx) / edge_px
        return 0.0


# ─────────────────────────────────────────────────────────────────────────────
#  Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _not_found() -> dict:
    return {
        "is_found":   False,
        "angle_x":    0.0,
        "angle_y":    0.0,
        "marker_id":  -1,
        "pixel_x":    0.0,
        "pixel_y":    0.0,
        "distance_m": 0.0,
    }


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()