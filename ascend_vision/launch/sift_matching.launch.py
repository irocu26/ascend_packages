import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import os

class SiftTrackerNode(Node):
    def __init__(self):
        super().__init__('sift_node')

        # 1. Declare and Read Parameters (Matching your launch file)
        self.declare_parameter('image_topic', '/rgbd_camera/image')
        self.declare_parameter('seed_images_dir', os.path.expanduser('~/ardu_ws/seed_images'))
        
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        seed_dir = self.get_parameter('seed_images_dir').get_parameter_value().string_value
        seed_path = os.path.join(seed_dir, 'target_cropped.jpg')

        # 2. Setup SIFT and Seed Image
        self.seed = cv2.imread(seed_path, cv2.IMREAD_GRAYSCALE)
        if self.seed is None:
            self.get_logger().error(f"CRITICAL: Could not load seed image at {seed_path}")
            raise FileNotFoundError(f"Missing seed image: {seed_path}")

        self.sift = cv2.SIFT_create()
        self.kp_seed, self.des_seed = self.sift.detectAndCompute(self.seed, None)
        self.des_seed = np.float32(self.des_seed)
        self.flann = cv2.FlannBasedMatcher(dict(algorithm=0, trees=5), dict(checks=50))
        
        self.get_logger().info(f"Seed loaded: {len(self.kp_seed)} keypoints. Tracking active.")

        # 3. Setup ROS2 Publishers, Subscribers, and Bridge
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, image_topic, self.image_callback, 10)
        
        # This is what the precision landing node will listen to!
        self.target_pub = self.create_publisher(Point, '/vision/target_position', 10)

        # 4. Tracking Memory Variables (Freeze-Frame EMA)
        self.lost_frames = 0
        self.MAX_LOST_FRAMES = 15
        self.last_M = None
        self.last_cx, self.last_cy = 0, 0
        self.last_inliers = 0
        self.smooth_x, self.smooth_y = 0.0, 0.0
        self.EMA_ALPHA = 0.2
        self.ever_locked = False
        
        # Hardcoded for initial desktop testing (Will upgrade to live AP telemetry later)
        self.TEST_ALTITUDE_M = 0.3 

    def calculate_local_coords(self, cx, cy, img_w, img_h, fov_deg, alt):
        fov_rad = math.radians(fov_deg)
        ground_w = 2 * alt * math.tan(fov_rad / 2)
        ground_h = ground_w * (img_h / img_w)
        gsd_x = ground_w / img_w
        gsd_y = ground_h / img_h
        offset_x = (cx - (img_w / 2)) * gsd_x
        offset_y = ((img_h / 2) - cy) * gsd_y
        return offset_x, offset_y

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        h_img, w_img = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp_frame, des_frame = self.sift.detectAndCompute(gray, None)

        current_frame_locked = False

        if des_frame is not None and len(kp_frame) > 4:
            des_frame = np.float32(des_frame)
            matches = self.flann.knnMatch(self.des_seed, des_frame, k=2)
            good = [m for m, n in matches if len([m, n]) == 2 and m.distance < 0.65 * n.distance]

            if len(good) >= 10:
                src_pts = np.float32([self.kp_seed[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp_frame[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
                
                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 4.0)
                
                if M is not None and mask is not None:
                    inliers = int(mask.sum())
                    if inliers >= 8:
                        current_frame_locked = True
                        self.ever_locked = True
                        self.last_M = M
                        inlier_pts = dst_pts[mask.ravel() == 1]
                        self.last_cx = float(np.mean(inlier_pts[:, 0, 0]))
                        self.last_cy = float(np.mean(inlier_pts[:, 0, 1]))
                        self.last_inliers = inliers

        # --- Logic & Data Publishing ---
        if current_frame_locked:
            self.lost_frames = 0
            box_color = (0, 255, 0)
            status_text = "TARGET LOCKED"
        else:
            self.lost_frames += 1
            if self.lost_frames < self.MAX_LOST_FRAMES:
                box_color = (0, 255, 255)
                status_text = "COASTING..."
            else:
                box_color = (0, 0, 255) 
                status_text = "TARGET LOST"

        if self.ever_locked and self.last_M is not None:
            if self.lost_frames < self.MAX_LOST_FRAMES:
                raw_x, raw_y = self.calculate_local_coords(self.last_cx, self.last_cy, w_img, h_img, 70.0, self.TEST_ALTITUDE_M)
                self.smooth_x = (self.EMA_ALPHA * raw_x) + ((1 - self.EMA_ALPHA) * self.smooth_x)
                self.smooth_y = (self.EMA_ALPHA * raw_y) + ((1 - self.EMA_ALPHA) * self.smooth_y)

                # PUBLISH THE COORDINATES TO THE FLIGHT CONTROLLER LOGIC!
                target_msg = Point()
                target_msg.x = float(self.smooth_x)
                target_msg.y = float(self.smooth_y)
                target_msg.z = 0.0 # Could be used for confidence/inliers later
                self.target_pub.publish(target_msg)

                # Draw UI Box
                h_seed, w_seed = self.seed.shape
                corners = np.float32([[0, 0], [0, h_seed - 1], [w_seed - 1, h_seed - 1], [w_seed - 1, 0]]).reshape(-1, 1, 2)
                dst = cv2.perspectiveTransform(corners, self.last_M)
                frame = cv2.polylines(frame, [np.int32(dst)], True, box_color, 3, cv2.LINE_AA)

            # Draw Static UI Text
            cv2.circle(frame, (int(self.last_cx), int(self.last_cy)), 5, box_color, -1)
            cv2.putText(frame, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, box_color, 3)
            cv2.putText(frame, f"Local X: {self.smooth_x:+.3f}m", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(frame, f"Local Y: {self.smooth_y:+.3f}m", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        else:
            cv2.putText(frame, "WAITING FOR TARGET...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100), 2)

        # Show the debug feed
        cv2.imshow("ROS2 SIFT Node", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SiftTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()