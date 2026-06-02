import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import glob
import time

class SiftMatcherNode(Node):
    def __init__(self):
        super().__init__('sift_matcher_node')

        # Declare parameters
        self.declare_parameter('seed_images_dir', os.path.expanduser('~/ardu_ws/seed_images'))
        self.declare_parameter('image_topic', '/rgbd_camera/image')
        self.declare_parameter('min_match_count', 4)
        self.declare_parameter('use_pysift', False)

        seed_dir = self.get_parameter('seed_images_dir').value
        image_topic = self.get_parameter('image_topic').value
        self.MIN_MATCH_COUNT = self.get_parameter('min_match_count').value
        use_pysift = self.get_parameter('use_pysift').value

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

        # Initialize FLANN matcher
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        # Frame throttle — skip frames to keep up with stream
        self.frame_count = 0
        self.process_every_n = 3  # process 1 out of every 3 frames
        self.get_logger().info("SIFT Matcher Node initialized and ready.")

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

        for filepath in sorted(image_files):
            img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
            if img is None:
                self.get_logger().warn(f"Failed to load image: {filepath}")
                continue

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

            self.seed_data.append({
                'name': os.path.basename(filepath),
                'img': img,
                'kp': kp,
                'des': np.float32(des),  # FLANN needs float32
            })
            self.get_logger().info(
                f"Loaded {os.path.basename(filepath)}: "
                f"{len(kp)} keypoints in {elapsed:.2f}s"
            )

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

            if M is not None:
                h, w = seed_img.shape
                pts = np.float32([
                    [0, 0], [0, h - 1],
                    [w - 1, h - 1], [w - 1, 0]
                ]).reshape(-1, 1, 2)
                dst = cv2.perspectiveTransform(pts, M)

                result_img = cv2.polylines(
                    result_img, [np.int32(dst)], True,
                    (0, 255, 0), 3, cv2.LINE_AA
                )
                cv2.putText(
                    result_img,
                    f"MATCH: {best_match_name} ({len(best_match_good)} pts)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
                )

            self.get_logger().info(
                f"MATCH FOUND: {best_match_name} "
                f"({len(best_match_good)} good matches)",
                throttle_duration_sec=1.0
            )
        else:
            n_good = len(best_match_good) if best_match_good else 0
            cv2.putText(
                result_img,
                f"No match (best: {n_good}/{self.MIN_MATCH_COUNT})",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
            )
            self.get_logger().info(
                f"No match (best: {n_good}/{self.MIN_MATCH_COUNT} needed)",
                throttle_duration_sec=2.0
            )

        # Publish result
        try:
            self.publisher.publish(self.bridge.cv2_to_imgmsg(result_img, "bgr8"))
        except Exception as e:
            self.get_logger().error(f"Publish error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SiftMatcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
