import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import glob
import re
import json
import time
import csv
from datetime import datetime
import math

class SiftMatcherNode(Node):
    def __init__(self):
        super().__init__('sift_matcher_node')

        # ── Existing Parameters ──
        self.declare_parameter('seed_images_dir', os.path.expanduser('~/ardu_ws/seed_images'))
        self.declare_parameter('image_topic', '/rgbd_camera/image')
        # CameraInfo for back-projecting a matched feature's pixel to a bearing
        # (#5). Should be the image topic's camera; falls back to a rough
        # image-size pinhole guess if it never arrives.
        self.declare_parameter('camera_info_topic', '/rgbd_camera/camera_info')
        # Min ratio-test matches required before we even attempt RANSAC (#2).
        # Raised from 4 — the bare minimum for a homography, with zero
        # outlier-rejection headroom — to the standard 10. Keep this >=
        # min_inliers below: you can't get more inliers than total good matches.
        self.declare_parameter('min_match_count', 10)
        self.declare_parameter('min_inliers', 10)
        self.declare_parameter('use_pysift', False)
        self.declare_parameter('match_debounce_s', 5.0)
        self.declare_parameter('match_result_topic', '/ascend/vision/match_result_str')
        self.declare_parameter('process_max_dim', 0)
        self.declare_parameter('square_resize', False)
        self.declare_parameter('debug_image_topic', 'sift_downsampled')
        
        # ── New Geolocation & Logging Parameters ──
        self.declare_parameter('log_dir', os.path.expanduser('~/ascend_flight_data'))
        self.declare_parameter('fov_h', 69.4) # Intel D435i Default FOV

        seed_dir = self.get_parameter('seed_images_dir').value
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.MIN_MATCH_COUNT = self.get_parameter('min_match_count').value
        self.min_inliers = self.get_parameter('min_inliers').value
        use_pysift = self.get_parameter('use_pysift').value
        self.match_debounce_s = self.get_parameter('match_debounce_s').value
        match_result_topic = self.get_parameter('match_result_topic').value
        self.process_max_dim = self.get_parameter('process_max_dim').value
        self.square_resize = self.get_parameter('square_resize').value
        debug_image_topic = self.get_parameter('debug_image_topic').value
        log_dir = self.get_parameter('log_dir').value
        self.FOV_H = self.get_parameter('fov_h').value

        # ── Geolocation State Variables ──
        self.drone_x, self.drone_y, self.drone_alt, self.drone_yaw = 0.0, 0.0, 3.0, 0.0
        self.pose_received = False
        self.EMA_ALPHA = 0.3
        self.smooth_x, self.smooth_y = 0.0, 0.0
        self.ema_initialized = False
        self.last_snapshot_time = 0.0

        # ── Setup Logging Directories ──
        os.makedirs(log_dir, exist_ok=True)
        self.snapshot_dir = os.path.join(log_dir, 'snapshots')
        os.makedirs(self.snapshot_dir, exist_ok=True)
        
        self.csv_path = os.path.join(log_dir, f'flight_detections_{int(time.time())}.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'seed_name', 'pixel_x', 'pixel_y', 'world_x_m', 'world_y_m', 'altitude_m', 'yaw_deg', 'inliers'])

        # ── Existing Init Logic ──
        self._last_pub_time = {}
        self.bridge = CvBridge()
        self.seed_data = []

        if use_pysift:
            from . import pysift
            self.pysift = pysift
            self.sift = None
            self.get_logger().info("Using pure-Python pysift")
        else:
            self.pysift = None
            self.sift = cv2.SIFT_create()
            self.get_logger().info("Using OpenCV SIFT")

        self.load_seed_images(seed_dir)

        if not self.seed_data:
            self.get_logger().warn(
                f"No seed images loaded from '{seed_dir}'. "
                "Node will not perform matching."
            )

        # ── ROS Subscriptions & Publishers ──
        self.get_logger().info(f"Subscribing to topic: {image_topic}")
        self.subscription = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.sub_pose = self.create_subscription(PoseStamped, '/ap/pose/filtered', self.pose_cb, 10) # NEW: EKF Pose

        # Camera intrinsics for back-projecting a matched feature's pixel to a
        # bearing (#5). Populated from CameraInfo; image-size fallback otherwise.
        self._fx = self._fy = self._cx = self._cy = None
        self.sub_cam_info = self.create_subscription(
            CameraInfo, camera_info_topic, self._cb_camera_info, 10)
        self.get_logger().info(f"Camera info topic: {camera_info_topic}")

        self.publisher = self.create_publisher(Image, 'sift_matches', 10)
        self.match_pub = self.create_publisher(String, match_result_topic, 10)
        self.debug_pub = self.create_publisher(Image, debug_image_topic, 10)
        
        self.pub_world = self.create_publisher(Point, '/vision/target_position', 10) # NEW: Geo Pub
        self.pub_pixel = self.create_publisher(Point, '/vision/target_pixel', 10)    # NEW: Geo Pub

        FLANN_INDEX_KDTREE = 0
        self.flann = cv2.FlannBasedMatcher(dict(algorithm=FLANN_INDEX_KDTREE, trees=5), dict(checks=50))
        self.frame_count = 0
        self.process_every_n = 3

    def _cb_camera_info(self, msg: CameraInfo):
        """Latch pinhole intrinsics (fx, fy, cx, cy) from CameraInfo."""
        k = msg.k
        self._fx, self._fy = k[0], k[4]
        self._cx, self._cy = k[2], k[5]

    def _intrinsics(self, shape):
        """(fx, fy, cx, cy) from CameraInfo, or a rough image-size fallback.

        The fallback assumes fx = fy = max(w, h) and the principal point at the
        image centre — only a coarse guess, so feature coordinates are
        approximate until real CameraInfo arrives.
        """
        if self._fx is not None:
            return self._fx, self._fy, self._cx, self._cy
        h, w = shape[:2]
        f = float(max(w, h))
        self.get_logger().warn(
            "No CameraInfo yet — using rough image-size intrinsics; feature "
            "coordinates will be approximate.",
            throttle_duration_sec=5.0,
        )
        return f, f, w / 2.0, h / 2.0

    # ── New Math Functions ──
    def quat_to_yaw(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def pose_cb(self, msg):
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y
        self.drone_alt = max(0.1, msg.pose.position.z) # Prevent Div by Zero
        q = msg.pose.orientation
        yaw_rad = self.quat_to_yaw(q.x, q.y, q.z, q.w)
        self.drone_yaw = math.degrees(yaw_rad)
        self.pose_received = True

    def calculate_world_coords(self, cx, cy, img_w, img_h):
        fov_rad = math.radians(self.FOV_H)
        ground_w = 2 * self.drone_alt * math.tan(fov_rad / 2)
        ground_h = ground_w * (img_h / img_w)
        gsd_x = ground_w / img_w
        gsd_y = ground_h / img_h
        offset_x = (cx - (img_w / 2)) * gsd_x
        offset_y = ((img_h / 2) - cy) * gsd_y
        yaw_rad = math.radians(self.drone_yaw)
        rx = offset_x * math.cos(yaw_rad) - offset_y * math.sin(yaw_rad)
        ry = offset_x * math.sin(yaw_rad) + offset_y * math.cos(yaw_rad)
        return self.drone_x + rx, self.drone_y + ry

    # ── Existing Functions (Untouched) ──
    def _downsample(self, img):
        h, w = img.shape[:2]
        if self.square_resize:
            target = self.process_max_dim if (self.process_max_dim and self.process_max_dim > 0) else 128
            resized = cv2.resize(img, (target, target), interpolation=cv2.INTER_AREA)
            return resized, target / float(w), target / float(h)
        if not self.process_max_dim or self.process_max_dim <= 0:
            return img, 1.0, 1.0
        long_side = max(h, w)
        if long_side <= self.process_max_dim:
            return img, 1.0, 1.0
        scale = self.process_max_dim / float(long_side)
        new_w = max(1, int(round(w * scale)))
        new_h = max(1, int(round(h * scale)))
        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
        return resized, scale, scale

    def compute_sift(self, gray_image):
        if self.sift is not None:
            kp, des = self.sift.detectAndCompute(gray_image, None)
            return kp, des
        else:
            kp, des = self.pysift.computeKeypointsAndDescriptors(gray_image)
            return kp, des

    def load_seed_images(self, seed_dir):
        if not seed_dir or not os.path.exists(seed_dir):
            self.get_logger().error(f"Invalid seed images directory: '{seed_dir}'")
            return
        valid_extensions = ['*.jpg', '*.jpeg', '*.png', '*.JPG', '*.JPEG', '*.PNG']
        image_files = []
        for ext in valid_extensions:
            image_files.extend(glob.glob(os.path.join(seed_dir, ext)))

        for load_index, filepath in enumerate(sorted(image_files)):
            img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
            if img is None: continue
            img, _, _ = self._downsample(img)
            kp, des = self.compute_sift(img)
            if des is None or len(kp) == 0: continue
            name = os.path.basename(filepath)
            seed_id = self._derive_seed_id(name, load_index)
            self.seed_data.append({
                'name': name, 'seed_id': seed_id, 'path': filepath, 
                'img': img, 'kp': kp, 'des': np.float32(des),
            })

    def _derive_seed_id(self, filename, load_index):
        m = re.search(r'\d+', filename)
        if m: return int(m.group())
        return load_index

    # ── Modified Image Callback ──
    def image_callback(self, msg):
        if not self.seed_data:
            return

        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray, scene_sx, scene_sy = self._downsample(gray)
        
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(gray, "mono8"))
        except: pass

        kp_scene, des_scene = self.compute_sift(gray)

        if des_scene is None or len(kp_scene) == 0:
            self.publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            return

        des_scene = np.float32(des_scene)

        best_match_name = None
        best_match_good = []
        best_match_seed = None

        for seed in self.seed_data:
            if seed['des'] is None: continue
            try:
                matches = self.flann.knnMatch(seed['des'], des_scene, k=2)
                good = [m for match in matches if len(match) == 2 for m, n in [match] if m.distance < 0.7 * n.distance]
                if len(good) > len(best_match_good):
                    best_match_good = good
                    best_match_seed = seed
                    best_match_name = seed['name']
            except: pass

        result_img = cv_image.copy()
        match_accepted = False
        n_inliers = 0
        feat_geom = None   # feature pixel + bearing, filled on an accepted match

        # Overlay text arrays
        overlay_lines = []
        status_color = (0, 0, 255) # Red default

        if best_match_seed and len(best_match_good) >= self.MIN_MATCH_COUNT:
            seed_img = best_match_seed['img']
            kp_seed = best_match_seed['kp']

            src_pts = np.float32([kp_seed[m.queryIdx].pt for m in best_match_good]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp_scene[m.trainIdx].pt for m in best_match_good]).reshape(-1, 1, 2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            n_inliers = int(mask.sum()) if mask is not None else 0

            if M is not None and n_inliers >= self.min_inliers:
                match_accepted = True
                status_color = (0, 255, 0) # Green

                # Draw Bounding Box
                h, w = seed_img.shape
                pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
                dst = cv2.perspectiveTransform(pts, M)
                if scene_sx != 1.0 or scene_sy != 1.0:
                    dst = dst / np.array([scene_sx, scene_sy], dtype=np.float32)
                
                result_img = cv2.polylines(result_img, [np.int32(dst)], True, status_color, 3, cv2.LINE_AA)
                
                # ── GEOLOCATION MATH & LOGGING ──
                # Calculate pixel center from the bounding box
                cx = float(np.mean(dst[:, 0, 0]))
                cy = float(np.mean(dst[:, 0, 1]))
                
                # Draw crosshair
                cv2.circle(result_img, (int(cx), int(cy)), 10, (0,0,255), -1)
                cv2.arrowedLine(result_img, (cv_image.shape[1]//2, cv_image.shape[0]//2), (int(cx), int(cy)), (0,255,255), 2)
                
                overlay_lines.append(f"MATCH: {best_match_name} ({n_inliers} inliers)")

                # Feature centroid in FULL-RES pixels and its bearing from the
                # image centre (#5). Same convention as aruco_detector so the FSM
                # can reuse precision_landing's pixel->ground maths
                # (angle_x +right of centre, angle_y +below centre).
                u, v = dst.reshape(-1, 2).mean(axis=0)
                fx, fy, ppx, ppy = self._intrinsics(cv_image.shape)
                feat_geom = {
                    'pixel_x': round(float(u), 1),
                    'pixel_y': round(float(v), 1),
                    'angle_x': round(float(np.arctan2(u - ppx, fx)), 5),
                    'angle_y': round(float(np.arctan2(v - ppy, fy)), 5),
                }

                if self.pose_received:
                    abs_x, abs_y = self.calculate_world_coords(cx, cy, cv_image.shape[1], cv_image.shape[0])
                    
                    # EMA Filter
                    if not self.ema_initialized:
                        self.smooth_x, self.smooth_y = abs_x, abs_y
                        self.ema_initialized = True
                    else:
                        self.smooth_x = self.EMA_ALPHA * abs_x + (1 - self.EMA_ALPHA) * self.smooth_x
                        self.smooth_y = self.EMA_ALPHA * abs_y + (1 - self.EMA_ALPHA) * self.smooth_y

                    # Publish topics
                    self.pub_world.publish(Point(x=self.smooth_x, y=self.smooth_y, z=0.0))
                    self.pub_pixel.publish(Point(x=cx, y=cy, z=0.0))

                    # Log to CSV
                    self.csv_writer.writerow([datetime.now().isoformat(), best_match_name, int(cx), int(cy), round(self.smooth_x, 4), round(self.smooth_y, 4), self.drone_alt, self.drone_yaw, n_inliers])

                    # Add to HUD
                    overlay_lines.extend([
                        f"East (X): {self.smooth_x:+.3f}m | North (Y): {self.smooth_y:+.3f}m",
                        f"Alt: {self.drone_alt:.1f}m | Yaw: {self.drone_yaw:.1f}deg"
                    ])

                    # Throttled Snapshot (Max 1 per second)
                    current_time = time.time()
                    if current_time - self.last_snapshot_time > 1.0:
                        # We save it without the overlay text to keep the raw image clean for the committee report
                        snap_name = os.path.join(self.snapshot_dir, f"lock_{best_match_name}_{int(current_time)}.jpg")
                        cv2.imwrite(snap_name, result_img)
                        self.last_snapshot_time = current_time

        if match_accepted:
            self.get_logger().info(
                f"MATCH FOUND: {best_match_name} "
                f"({n_inliers} RANSAC inliers / {len(best_match_good)} good)",
                throttle_duration_sec=1.0
            )
            # Tell the FSM about the match (debounced per seed_id), including the
            # feature bearing so it can back-project to an arena coordinate.
            self._publish_match(best_match_seed, n_inliers, feat_geom)
        else:
            self.ema_initialized = False # Reset EMA on loss
            n_good = len(best_match_good) if best_match_good else 0
            reason = f"{n_inliers} inliers < {self.min_inliers}" if n_good >= self.MIN_MATCH_COUNT else f"{n_good}/{self.MIN_MATCH_COUNT} good"
            overlay_lines.append(f"No match ({reason})")

        # ── Draw Clean HUD (Outline Trick) ──
        # Center Drone Crosshair
        ch_y, ch_x = cv_image.shape[0]//2, cv_image.shape[1]//2
        cv2.line(result_img, (ch_x-20, ch_y), (ch_x+20, ch_y), (255,0,0), 2)
        cv2.line(result_img, (ch_x, ch_y-20), (ch_x, ch_y+20), (255,0,0), 2)

        for i, text in enumerate(overlay_lines):
            y = 30 + (i * 30)
            color = status_color if i == 0 else (0, 255, 255)
            cv2.putText(result_img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(result_img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 1, cv2.LINE_AA)

        try:
            self.publisher.publish(self.bridge.cv2_to_imgmsg(result_img, "bgr8"))
        except: pass

    def _publish_match(self, seed, n_inliers, feat_geom=None):
        """Publish a structured match result to the FSM, debounced per seed_id.

        FSM expects std_msgs/String JSON: {seed_id, confidence, hd_path}, plus
        (#5) the feature bearing {pixel_x, pixel_y, angle_x, angle_y} so the FSM
        can back-project it to an arena coordinate instead of using the drone's
        own position.
        """
        seed_id = seed['seed_id']
        now = time.time()
        last = self._last_pub_time.get(seed_id, 0.0)
        if now - last < self.match_debounce_s: return
        self._last_pub_time[seed_id] = now
        denom = float(max(1, 2 * self.min_inliers))
        confidence = max(0.0, min(1.0, n_inliers / denom))

        payload = {
            'seed_id':    int(seed_id),
            'confidence': round(float(confidence), 3),
            'hd_path':    seed['path'],
        }
        if feat_geom:
            payload.update(feat_geom)
        msg = String()
        msg.data = json.dumps(payload)
        self.match_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SiftMatcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()