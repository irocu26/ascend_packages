# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import os
# import glob
# import re
# import json
# import time

# class SiftMatcherNode(Node):
#     def __init__(self):
#         super().__init__('sift_matcher_node')

#         # Declare parameters
#         self.declare_parameter('seed_images_dir', os.path.expanduser('~/ardu_ws/seed_images'))
#         self.declare_parameter('image_topic', '/rgbd_camera/image')
#         self.declare_parameter('min_match_count', 4)
#         self.declare_parameter('use_pysift', False)
#         # How long (s) to suppress re-publishing the SAME seed_id as a match.
#         self.declare_parameter('match_debounce_s', 5.0)
#         # match_result topic the FSM listens on (String JSON fallback).
#         self.declare_parameter('match_result_topic', '/ascend/vision/match_result_str')

#         seed_dir = self.get_parameter('seed_images_dir').value
#         image_topic = self.get_parameter('image_topic').value
#         self.MIN_MATCH_COUNT = self.get_parameter('min_match_count').value
#         use_pysift = self.get_parameter('use_pysift').value
#         self.match_debounce_s = self.get_parameter('match_debounce_s').value
#         match_result_topic = self.get_parameter('match_result_topic').value

#         # Debounce bookkeeping: seed_id -> last publish wall-clock time
#         self._last_pub_time = {}

#         self.bridge = CvBridge()
#         self.seed_data = []  # List of dicts: 'name', 'img', 'kp', 'des'

#         # Choose SIFT backend
#         if use_pysift:
#             from . import pysift
#             self.pysift = pysift
#             self.sift = None
#             self.get_logger().info("Using pure-Python pysift (SLOW — expect seconds per frame)")
#         else:
#             self.pysift = None
#             self.sift = cv2.SIFT_create()
#             self.get_logger().info("Using OpenCV SIFT (fast C++ backend)")

#         # Load seed images
#         self.load_seed_images(seed_dir)

#         if not self.seed_data:
#             self.get_logger().warn(
#                 f"No seed images loaded from '{seed_dir}'. "
#                 "Node will not perform matching."
#             )

#         self.get_logger().info(f"Subscribing to topic: {image_topic}")
#         self.subscription = self.create_subscription(
#             Image,
#             image_topic,
#             self.image_callback,
#             10)

#         self.publisher = self.create_publisher(Image, 'sift_matches', 10)

#         # Structured match result for the FSM (std_msgs/String JSON).
#         self.match_pub = self.create_publisher(String, match_result_topic, 10)
#         self.get_logger().info(f"Publishing match results to: {match_result_topic}")

#         # Initialize FLANN matcher
#         FLANN_INDEX_KDTREE = 0
#         index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
#         search_params = dict(checks=50)
#         self.flann = cv2.FlannBasedMatcher(index_params, search_params)

#         # Frame throttle — skip frames to keep up with stream
#         self.frame_count = 0
#         self.process_every_n = 3  # process 1 out of every 3 frames
#         self.get_logger().info("SIFT Matcher Node initialized and ready.")

#     def compute_sift(self, gray_image):
#         """Compute SIFT keypoints and descriptors using the chosen backend."""
#         if self.sift is not None:
#             # OpenCV built-in SIFT (fast)
#             kp, des = self.sift.detectAndCompute(gray_image, None)
#             return kp, des
#         else:
#             # Pure-Python pysift (slow)
#             kp, des = self.pysift.computeKeypointsAndDescriptors(gray_image)
#             return kp, des

#     def load_seed_images(self, seed_dir):
#         if not seed_dir or not os.path.exists(seed_dir):
#             self.get_logger().error(f"Invalid seed images directory: '{seed_dir}'")
#             return

#         valid_extensions = ['*.jpg', '*.jpeg', '*.png', '*.JPG', '*.JPEG', '*.PNG']
#         image_files = []
#         for ext in valid_extensions:
#             image_files.extend(glob.glob(os.path.join(seed_dir, ext)))

#         self.get_logger().info(f"Found {len(image_files)} image(s) in {seed_dir}")

#         for load_index, filepath in enumerate(sorted(image_files)):
#             img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
#             if img is None:
#                 self.get_logger().warn(f"Failed to load image: {filepath}")
#                 continue

#             self.get_logger().info(f"Computing SIFT for {filepath}...")
#             t0 = time.time()
#             kp, des = self.compute_sift(img)
#             elapsed = time.time() - t0

#             if des is None or len(kp) == 0:
#                 self.get_logger().warn(
#                     f"No keypoints found in {filepath} — skipping. "
#                     "Try using a more textured seed image."
#                 )
#                 continue

#             name = os.path.basename(filepath)
#             seed_id = self._derive_seed_id(name, load_index)
#             self.seed_data.append({
#                 'name': name,
#                 'seed_id': seed_id,
#                 'path': filepath,          # used as hd_path in the match result
#                 'img': img,
#                 'kp': kp,
#                 'des': np.float32(des),    # FLANN needs float32
#             })
#             self.get_logger().info(
#                 f"Loaded {name} (seed_id={seed_id}): "
#                 f"{len(kp)} keypoints in {elapsed:.2f}s"
#             )

#     def _derive_seed_id(self, filename, load_index):
#         """Map a seed filename to an integer seed_id.

#         Prefer the first number embedded in the filename (e.g. 'seed_3.png' -> 3,
#         '12.jpg' -> 12). If the name has no digits (e.g. 'Box.png'), fall back to
#         the load order index so every seed still gets a unique, stable id.
#         """
#         m = re.search(r'\d+', filename)
#         if m:
#             return int(m.group())
#         return load_index

#     def image_callback(self, msg):
#         if not self.seed_data:
#             return

#         # Throttle: skip frames to avoid lag
#         self.frame_count += 1
#         if self.frame_count % self.process_every_n != 0:
#             return

#         self.get_logger().debug("Processing frame...", throttle_duration_sec=2.0)

#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         except Exception as e:
#             self.get_logger().error(f"cv_bridge error: {e}")
#             return

#         gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

#         # Compute SIFT for current frame
#         t0 = time.time()
#         kp_scene, des_scene = self.compute_sift(gray)
#         sift_time = time.time() - t0

#         if des_scene is None or len(kp_scene) == 0:
#             self.get_logger().info("No keypoints in current frame.", throttle_duration_sec=2.0)
#             self.publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#             return

#         des_scene = np.float32(des_scene)

#         self.get_logger().info(
#             f"Frame SIFT: {len(kp_scene)} keypoints in {sift_time:.2f}s",
#             throttle_duration_sec=2.0
#         )

#         best_match_name = None
#         best_match_good = []
#         best_match_seed = None

#         # Try to match against each seed image
#         for seed in self.seed_data:
#             if seed['des'] is None:
#                 continue

#             try:
#                 matches = self.flann.knnMatch(seed['des'], des_scene, k=2)

#                 good = []
#                 for match in matches:
#                     if len(match) == 2:
#                         m, n = match
#                         if m.distance < 0.7 * n.distance:
#                             good.append(m)

#                 if len(good) > len(best_match_good):
#                     best_match_good = good
#                     best_match_seed = seed
#                     best_match_name = seed['name']
#             except Exception as e:
#                 self.get_logger().warn(f"Matching error with {seed['name']}: {e}")

#         # Draw result
#         result_img = cv_image.copy()

#         if best_match_seed and len(best_match_good) >= self.MIN_MATCH_COUNT:
#             seed_img = best_match_seed['img']
#             kp_seed = best_match_seed['kp']

#             src_pts = np.float32(
#                 [kp_seed[m.queryIdx].pt for m in best_match_good]
#             ).reshape(-1, 1, 2)
#             dst_pts = np.float32(
#                 [kp_scene[m.trainIdx].pt for m in best_match_good]
#             ).reshape(-1, 1, 2)

#             M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

#             if M is not None:
#                 h, w = seed_img.shape
#                 pts = np.float32([
#                     [0, 0], [0, h - 1],
#                     [w - 1, h - 1], [w - 1, 0]
#                 ]).reshape(-1, 1, 2)
#                 dst = cv2.perspectiveTransform(pts, M)

#                 result_img = cv2.polylines(
#                     result_img, [np.int32(dst)], True,
#                     (0, 255, 0), 3, cv2.LINE_AA
#                 )
#                 cv2.putText(
#                     result_img,
#                     f"MATCH: {best_match_name} ({len(best_match_good)} pts)",
#                     (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
#                 )

#             self.get_logger().info(
#                 f"MATCH FOUND: {best_match_name} "
#                 f"({len(best_match_good)} good matches)",
#                 throttle_duration_sec=1.0
#             )

#             # Tell the FSM about the match (debounced per seed_id).
#             self._publish_match(best_match_seed, len(best_match_good))
#         else:
#             n_good = len(best_match_good) if best_match_good else 0
#             cv2.putText(
#                 result_img,
#                 f"No match (best: {n_good}/{self.MIN_MATCH_COUNT})",
#                 (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
#             )
#             self.get_logger().info(
#                 f"No match (best: {n_good}/{self.MIN_MATCH_COUNT} needed)",
#                 throttle_duration_sec=2.0
#             )

#         # Publish result
#         try:
#             self.publisher.publish(self.bridge.cv2_to_imgmsg(result_img, "bgr8"))
#         except Exception as e:
#             self.get_logger().error(f"Publish error: {e}")

#     def _publish_match(self, seed, n_good):
#         """Publish a structured match result to the FSM, debounced per seed_id.

#         FSM expects std_msgs/String JSON: {seed_id, confidence, hd_path}.
#         """
#         seed_id = seed['seed_id']
#         now = time.time()
#         last = self._last_pub_time.get(seed_id, 0.0)
#         if now - last < self.match_debounce_s:
#             return  # suppress repeated publishes of the same seed
#         self._last_pub_time[seed_id] = now

#         # Confidence heuristic: scale good-match count toward 1.0. At 3x the
#         # minimum threshold we call it fully confident.
#         confidence = max(0.0, min(1.0, n_good / float(3 * self.MIN_MATCH_COUNT)))

#         payload = {
#             'seed_id':    int(seed_id),
#             'confidence': round(float(confidence), 3),
#             'hd_path':    seed['path'],
#         }
#         msg = String()
#         msg.data = json.dumps(payload)
#         self.match_pub.publish(msg)
#         self.get_logger().info(f"Published match result → {payload}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = SiftMatcherNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import glob
import re
import json
import time

class SiftMatcherNode(Node):
    def __init__(self):
        super().__init__('sift_matcher_node')

        # Declare parameters
        self.declare_parameter('seed_images_dir', os.path.expanduser('~/ardu_ws/seed_images'))
        self.declare_parameter('image_topic', '/rgbd_camera/image')
        # Min ratio-test matches required before we even attempt RANSAC (#2).
        # Raised from 4 — the bare minimum for a homography, with zero
        # outlier-rejection headroom — to the standard 10. Keep this >=
        # min_inliers below: you can't get more inliers than total good matches.
        self.declare_parameter('min_match_count', 10)
        # Geometric-verification gate (#1): a seed is accepted ONLY if its
        # ratio-test matches fit a single homography with at least this many
        # RANSAC inliers. Raw ratio-test count alone is not enough — a few
        # geometrically-inconsistent matches can pass the ratio test.
        self.declare_parameter('min_inliers', 10)
        self.declare_parameter('use_pysift', False)
        # How long (s) to suppress re-publishing the SAME seed_id as a match.
        self.declare_parameter('match_debounce_s', 5.0)
        # match_result topic the FSM listens on (String JSON fallback).
        self.declare_parameter('match_result_topic', '/ascend/vision/match_result_str')
        # Optional SPEED cap: shrink the scene so its long side is at most this
        # many pixels before SIFT (aspect preserved). 0 = OFF = full native
        # resolution, which is the DEFAULT (#3). Do NOT shrink the scene down to
        # ~the seed size: the feature is a small fraction of the frame, so
        # downsampling the whole frame collapses a ~64px feature to ~6-13px and
        # SIFT can no longer match it. Only set a cap once you've confirmed the
        # feature still spans >~40px after it (watch the sift_matches box size
        # and the per-frame keypoint counts).
        self.declare_parameter('process_max_dim', 0)
        # Resize convention (#4). SIFT descriptors are NOT invariant to
        # anisotropic (non-uniform) scaling, so the seed and the scene MUST be
        # resized the same way the 128x128 references were generated:
        #   False (default) = aspect-preserving uniform scale (no distortion).
        #     Use when the references are undistorted feature crops.
        #   True = anamorphic squash to a square (e.g. 1280x720 -> NxN), i.e. the
        #     booklet's literal "1280x720 -> 128x128". WARNING: squashing the
        #     whole frame also shrinks a small feature into oblivion (see #3), so
        #     this is only valid when the feature fills the frame.
        # Either way the SAME transform is applied to seeds and scene.
        self.declare_parameter('square_resize', False)
        # Debug topic carrying the downsampled image SIFT actually sees.
        self.declare_parameter('debug_image_topic', 'sift_downsampled')

        seed_dir = self.get_parameter('seed_images_dir').value
        image_topic = self.get_parameter('image_topic').value
        self.MIN_MATCH_COUNT = self.get_parameter('min_match_count').value
        self.min_inliers = self.get_parameter('min_inliers').value
        use_pysift = self.get_parameter('use_pysift').value
        self.match_debounce_s = self.get_parameter('match_debounce_s').value
        match_result_topic = self.get_parameter('match_result_topic').value
        self.process_max_dim = self.get_parameter('process_max_dim').value
        self.square_resize = self.get_parameter('square_resize').value
        debug_image_topic = self.get_parameter('debug_image_topic').value

        # Debounce bookkeeping: seed_id -> last publish wall-clock time
        self._last_pub_time = {}

        self.bridge = CvBridge()
        self.seed_data = []  # List of dicts: 'name', 'img', 'kp', 'des'

        # Choose SIFT backend
        if use_pysift:
            from . import pysift
            self.pysift = pysift
            self.sift = None
            self.get_logger().info("Using pure-Python pysift (SLOW — expect seconds per frame)")
        else:
            self.pysift = None
            self.sift = cv2.SIFT_create()
            self.get_logger().info("Using OpenCV SIFT (fast C++ backend)")

        # Load seed images
        self.load_seed_images(seed_dir)

        if not self.seed_data:
            self.get_logger().warn(
                f"No seed images loaded from '{seed_dir}'. "
                "Node will not perform matching."
            )

        self.get_logger().info(f"Subscribing to topic: {image_topic}")
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10)

        self.publisher = self.create_publisher(Image, 'sift_matches', 10)

        # Structured match result for the FSM (std_msgs/String JSON).
        self.match_pub = self.create_publisher(String, match_result_topic, 10)
        self.get_logger().info(f"Publishing match results to: {match_result_topic}")

        # Debug stream: the downsampled image SIFT runs on.
        self.debug_pub = self.create_publisher(Image, debug_image_topic, 10)
        if self.square_resize:
            _sq = self.process_max_dim if (self.process_max_dim and self.process_max_dim > 0) else 128
            _res_desc = f"anamorphic squash to {_sq}x{_sq} (square_resize)"
            self.get_logger().warn(
                "square_resize=True: the whole frame is squashed to a square, "
                "which shrinks small features into oblivion (#3). Only use this "
                "if the feature fills the frame AND your references were squashed "
                "from 1280x720 the same way."
            )
        else:
            _res_desc = (
                f"capped to {self.process_max_dim}px long-side"
                if self.process_max_dim and self.process_max_dim > 0
                else "full native resolution"
            )
        self.get_logger().info(
            f"Scene processed at {_res_desc}; "
            f"debug image (what SIFT sees) on: {debug_image_topic}"
        )

        # Initialize FLANN matcher
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        # Frame throttle — skip frames to keep up with stream
        self.frame_count = 0
        self.process_every_n = 3  # process 1 out of every 3 frames
        self.get_logger().info("SIFT Matcher Node initialized and ready.")

    def _downsample(self, img):
        """Resize an image for SIFT, returning (resized, sx, sy).

        sx, sy = resized/original per axis; multiply resized coords by
        (1/sx, 1/sy) to map back to the original. In the default uniform
        (aspect-preserving) mode sx == sy. INTER_AREA area-averages, which
        avoids aliasing when shrinking.

        square_resize=True does an anamorphic squash to a square target
        (process_max_dim, or 128 if that's 0), replicating the booklet's literal
        1280x720 -> 128x128 so the scene is distorted the same way as squashed
        references (#4). sx != sy in that case.
        """
        h, w = img.shape[:2]

        if self.square_resize:
            target = self.process_max_dim if (self.process_max_dim and self.process_max_dim > 0) else 128
            resized = cv2.resize(img, (target, target), interpolation=cv2.INTER_AREA)
            return resized, target / float(w), target / float(h)

        # Uniform, aspect-preserving (default).
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
        """Compute SIFT keypoints and descriptors using the chosen backend."""
        if self.sift is not None:
            # OpenCV built-in SIFT (fast)
            kp, des = self.sift.detectAndCompute(gray_image, None)
            return kp, des
        else:
            # Pure-Python pysift (slow)
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

        self.get_logger().info(f"Found {len(image_files)} image(s) in {seed_dir}")

        for load_index, filepath in enumerate(sorted(image_files)):
            img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
            if img is None:
                self.get_logger().warn(f"Failed to load image: {filepath}")
                continue

            # Keep seeds at their native (low) resolution: a 128x128 LR
            # reference stays 128x128, so the feature's scale in the seed
            # (~the whole image) is comparable to its scale in the full-res
            # scene (a small patch). With process_max_dim=0 (default) this is a
            # no-op. Provide seeds at the scale the feature appears — NOT
            # full-frame HD photos, which won't scale-match the small scene
            # feature.
            img, _, _ = self._downsample(img)

            self.get_logger().info(f"Computing SIFT for {filepath}...")
            t0 = time.time()
            kp, des = self.compute_sift(img)
            elapsed = time.time() - t0

            if des is None or len(kp) == 0:
                self.get_logger().warn(
                    f"No keypoints found in {filepath} — skipping. "
                    "Try using a more textured seed image."
                )
                continue

            name = os.path.basename(filepath)
            seed_id = self._derive_seed_id(name, load_index)
            self.seed_data.append({
                'name': name,
                'seed_id': seed_id,
                'path': filepath,          # used as hd_path in the match result
                'img': img,
                'kp': kp,
                'des': np.float32(des),    # FLANN needs float32
            })
            self.get_logger().info(
                f"Loaded {name} (seed_id={seed_id}): "
                f"{len(kp)} keypoints in {elapsed:.2f}s"
            )

    def _derive_seed_id(self, filename, load_index):
        """Map a seed filename to an integer seed_id.

        Prefer the first number embedded in the filename (e.g. 'seed_3.png' -> 3,
        '12.jpg' -> 12). If the name has no digits (e.g. 'Box.png'), fall back to
        the load order index so every seed still gets a unique, stable id.
        """
        m = re.search(r'\d+', filename)
        if m:
            return int(m.group())
        return load_index

    def image_callback(self, msg):
        if not self.seed_data:
            return

        # Throttle: skip frames to avoid lag
        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return

        self.get_logger().debug("Processing frame...", throttle_duration_sec=2.0)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Downsample before SIFT. (scene_sx, scene_sy) map processed-image coords
        # back to the full-res frame (divide by them) so the overlay lands right.
        gray, scene_sx, scene_sy = self._downsample(gray)

        # Publish the downsampled image SIFT actually sees (debug stream).
        try:
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(gray, "mono8")
            )
        except Exception as e:
            self.get_logger().warn(f"Debug image publish error: {e}")

        # Compute SIFT for current frame
        t0 = time.time()
        kp_scene, des_scene = self.compute_sift(gray)
        sift_time = time.time() - t0

        if des_scene is None or len(kp_scene) == 0:
            self.get_logger().info("No keypoints in current frame.", throttle_duration_sec=2.0)
            self.publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            return

        des_scene = np.float32(des_scene)

        self.get_logger().info(
            f"Frame SIFT: {len(kp_scene)} keypoints in {sift_time:.2f}s",
            throttle_duration_sec=2.0
        )

        best_match_name = None
        best_match_good = []
        best_match_seed = None

        # Try to match against each seed image
        for seed in self.seed_data:
            if seed['des'] is None:
                continue

            try:
                matches = self.flann.knnMatch(seed['des'], des_scene, k=2)

                good = []
                for match in matches:
                    if len(match) == 2:
                        m, n = match
                        if m.distance < 0.7 * n.distance:
                            good.append(m)

                if len(good) > len(best_match_good):
                    best_match_good = good
                    best_match_seed = seed
                    best_match_name = seed['name']
            except Exception as e:
                self.get_logger().warn(f"Matching error with {seed['name']}: {e}")

        # Draw result
        result_img = cv_image.copy()

        # ── Geometric verification gates the match (#1) ─────────────────────
        # Ratio-test matches alone are NOT enough: a handful of geometrically
        # inconsistent matches can pass the ratio test and fire a false match.
        # Accept a seed only if its matches fit ONE homography with enough
        # RANSAC inliers. The homography + inlier count is the accept/reject
        # gate now, not just a drawing aid.
        match_accepted = False
        n_inliers = 0

        if best_match_seed and len(best_match_good) >= self.MIN_MATCH_COUNT:
            seed_img = best_match_seed['img']
            kp_seed = best_match_seed['kp']

            src_pts = np.float32(
                [kp_seed[m.queryIdx].pt for m in best_match_good]
            ).reshape(-1, 1, 2)
            dst_pts = np.float32(
                [kp_scene[m.trainIdx].pt for m in best_match_good]
            ).reshape(-1, 1, 2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            n_inliers = int(mask.sum()) if mask is not None else 0

            if M is not None and n_inliers >= self.min_inliers:
                match_accepted = True

                h, w = seed_img.shape
                pts = np.float32([
                    [0, 0], [0, h - 1],
                    [w - 1, h - 1], [w - 1, 0]
                ]).reshape(-1, 1, 2)
                dst = cv2.perspectiveTransform(pts, M)

                # Homography is in processed-scene coords; scale corners back up
                # (per-axis, since square_resize is anamorphic) so the box lands
                # on the full-res frame we draw on.
                if scene_sx != 1.0 or scene_sy != 1.0:
                    dst = dst / np.array([scene_sx, scene_sy], dtype=np.float32)

                result_img = cv2.polylines(
                    result_img, [np.int32(dst)], True,
                    (0, 255, 0), 3, cv2.LINE_AA
                )
                cv2.putText(
                    result_img,
                    f"MATCH: {best_match_name} ({n_inliers} inliers)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
                )

        if match_accepted:
            self.get_logger().info(
                f"MATCH FOUND: {best_match_name} "
                f"({n_inliers} RANSAC inliers / {len(best_match_good)} good)",
                throttle_duration_sec=1.0
            )
            # Tell the FSM about the match (debounced per seed_id).
            self._publish_match(best_match_seed, n_inliers)
        else:
            n_good = len(best_match_good) if best_match_good else 0
            if n_good >= self.MIN_MATCH_COUNT:
                reason = f"{n_inliers} inliers < {self.min_inliers}"   # geometry rejected
            else:
                reason = f"{n_good}/{self.MIN_MATCH_COUNT} good matches"
            cv2.putText(
                result_img,
                f"No match ({reason})",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
            )
            self.get_logger().info(
                f"No match ({reason})",
                throttle_duration_sec=2.0
            )

        # Publish result
        try:
            self.publisher.publish(self.bridge.cv2_to_imgmsg(result_img, "bgr8"))
        except Exception as e:
            self.get_logger().error(f"Publish error: {e}")

    def _publish_match(self, seed, n_inliers):
        """Publish a structured match result to the FSM, debounced per seed_id.

        FSM expects std_msgs/String JSON: {seed_id, confidence, hd_path}.
        """
        seed_id = seed['seed_id']
        now = time.time()
        last = self._last_pub_time.get(seed_id, 0.0)
        if now - last < self.match_debounce_s:
            return  # suppress repeated publishes of the same seed
        self._last_pub_time[seed_id] = now

        # Confidence from RANSAC inliers (geometric verification already passed
        # to get here). Fully confident at 2x the acceptance threshold.
        denom = float(max(1, 2 * self.min_inliers))
        confidence = max(0.0, min(1.0, n_inliers / denom))

        payload = {
            'seed_id':    int(seed_id),
            'confidence': round(float(confidence), 3),
            'hd_path':    seed['path'],
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.match_pub.publish(msg)
        self.get_logger().info(f"Published match result → {payload}")


def main(args=None):
    rclpy.init(args=args)
    node = SiftMatcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
