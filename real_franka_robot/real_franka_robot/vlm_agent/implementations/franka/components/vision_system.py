import rclpy
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as PILImage  # Renamed to avoid confusion with sensor_msgs
from geometry_msgs.msg import Point
from scipy.spatial.transform import Rotation as R

class VisionSystem:
    def __init__(self, node: Node):
        self.node = node
        self.logger = self.node.get_logger().get_child("vision_system")
        self.bridge = CvBridge()

        # Topic config
        self.image_topic = '/camera/camera/color/image_raw'
        self.info_topic = '/camera/camera/color/camera_info'

        # Frame Color Optical
        self.color_optical_frame = "camera_color_optical_frame"

        # State variables
        self.camera_intrinsics = None  # K Matrix
        self.distortion_coeffs = None  # D Coeffs
        self.latest_cv_image = None

        self.load_camera_info()

        self.logger.debug("Vision System Initialized. Waiting for Camera Info...")

    def load_camera_info(self, timeout_sec=5.0) -> bool:
        """
        BLOCKING (but safe) function that downloads camera parameters.
        Does everything here: subscribe -> wait -> process -> unsubscribe.
        """
        if self.camera_intrinsics is not None:
            return True  # Already loaded

        self.logger.debug("Waiting for Camera Info parameters...")

        # 1. Future to capture message
        future = Future()

        # 2. Subscriber uses a lambda to fill future directly
        sub = self.node.create_subscription(
            CameraInfo,
            self.info_topic,
            lambda msg: future.set_result(msg),
            1
        )

        try:
            # 3. Spin until message arrives
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

            if future.done():
                msg = future.result()
                self.camera_intrinsics = np.array(msg.k).reshape(3, 3)
                self.dist_coeffs = np.array(msg.d)
                self.logger.debug("Camera parameters loaded successfully.")
                return True
            else:
                self.logger.error("Timeout: Camera Info not received.")
                return False

        except Exception as e:
            self.logger.error(f"Error loading info: {e}")
            return False

        finally:
            # 4. Immediate cleanup
            self.node.destroy_subscription(sub)

    def take_snapshot(self, timeout_sec=5.0):
        """
        Takes a snapshot from the camera topic.
        """
        self.logger.debug("Requesting snapshot...")
        future = Future()

        sub = self.node.create_subscription(
            Image,
            self.image_topic,
            lambda msg: future.set_result(msg),
            1
        )

        try:
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

            if future.done():
                return future.result()
            else:
                self.logger.error("Timeout snapshot.")
                return None
        finally:
            self.node.destroy_subscription(sub)

    def image_to_cv(self, image: Image):
        cv_img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        return cv_img

    def cv_to_pil(self, cv_image):
        """
        Static or instance helper to convert final image.
        To be called AFTER performing cv2.circle/rectangle.
        """
        # OpenCV uses BGR, Gemini (and PIL) wants RGB
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(rgb_image)
        return pil_image

    def project_3d_to_2d(self, point_input):
        """
        Projects a 3D point to 2D using OpenCV.
        Accepts both ROS objects (Point/Vector3) and Numpy Arrays/Lists.

        :param point_input: geometry_msgs/Point, geometry_msgs/Vector3,
                            or numpy.array([x,y,z]) / list [x,y,z]
        :return: (u, v) tuple of integers, or None if it fails
        """
        # 0. Preliminary checks
        if self.camera_intrinsics is None:
            self.logger.error("Intrinsic matrix K not set!")
            return None

        # 1. Input Normalization: Extract x, y, z regardless of type
        try:
            if hasattr(point_input, 'x') and hasattr(point_input, 'y') and hasattr(point_input, 'z'):
                # ROS Message case (Point, Vector3, Pose.position)
                x, y, z = point_input.x, point_input.y, point_input.z
            else:
                # Numpy Array, List or Tuple case
                # Accept array shape (3,) or (3,1)
                flat_pt = np.array(point_input).flatten()
                if len(flat_pt) < 3:
                    raise ValueError(f"Input must have at least 3 coordinates, received: {flat_pt}")
                x, y, z = flat_pt[0], flat_pt[1], flat_pt[2]
        except Exception as e:
            self.logger.error(f"Input format error in project_3d_to_2d: {e}")
            return None

        # 2. Prepare array for OpenCV (float32)
        points_3d = np.array([x, y, z], dtype=np.float32)

        # 3. Camera Parameters
        K = self.camera_intrinsics
        D = self.distortion_coeffs if self.distortion_coeffs is not None else np.zeros(5)

        # Null vectors (point is already in camera frame)
        r_vec = np.zeros((3, 1))
        t_vec = np.zeros((3, 1))

        # 4. Projection
        try:
            image_points, _ = cv2.projectPoints(points_3d, r_vec, t_vec, K, D)

            # OpenCV returns shape (N, 1, 2), we have N=1
            u = int(image_points[0][0][0])
            v = int(image_points[0][0][1])
            return (u, v)

        except cv2.error as e:
            self.logger.error(f"OpenCV projectPoints error: {e}")
            return None

    def annotate_image_with_points(self, cv_image, points_text_map):
        """
        Draws points and text on an image.

        Args:
            cv_image: OpenCV Image (numpy array).
            points_text_map: Dictionary in format { (u, v): "Text to write" }
                            where (u, v) are integer tuples.

        Returns:
            annotated_img: The image with drawings on top.
        """
        annotated_img = cv_image.copy()

        for point, text in points_text_map.items():
            self.logger.debug(f"Point:{point}. Text: '{text}'")
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            font_thickness = 1

            # Calculation of background dimensions
            (text_w, text_h), baseline = cv2.getTextSize(text, font, font_scale, font_thickness)

            # Text position (slightly shifted from axis origin to not cover them)
            text_x = point[0] + 15
            text_y = point[1] - 15

            # Solid Black Background
            cv2.rectangle(annotated_img,
                          (text_x - 2, text_y - text_h - 2),
                          (text_x + text_w + 2, text_y + baseline + 2),
                          (0, 0, 0), cv2.FILLED)

            # White Text
            cv2.putText(annotated_img, text, (text_x, text_y), font, font_scale, (255, 255, 255), font_thickness,
                        cv2.LINE_AA)

        return annotated_img

    def draw_local_axes(self, cv_image, object_pose_in_camera_frame, axis_length=0.03):
        """
        PRO Version: Uses cv2.drawFrameAxes (ArUco style).
        Requires access to self.vision.camera_intrinsics and distortion_coeffs.
        """

        # 0. Safety checks
        if self.camera_intrinsics is None:
            self.logger.error("Missing K matrix, impossible to use drawFrameAxes")
            return cv_image

        # Polymorphic input handling (Pose or PoseStamped)
        if hasattr(object_pose_in_camera_frame, 'pose'):
            pose = object_pose_in_camera_frame.pose
        else:
            pose = object_pose_in_camera_frame

        # 1. TVEC Preparation (Translation)
        # Simply x,y,z position (since we are already in camera frame)
        tvec = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float32)

        # 2. RVEC Preparation (Rodrigues Rotation)
        # Convert Quaternion -> Matrix -> Rodrigues
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rot_matrix = R.from_quat(q).as_matrix()

        # cv2.Rodrigues converts 3x3 matrix to 3x1 vector (rvec)
        rvec, _ = cv2.Rodrigues(rot_matrix)

        # 3. Camera Parameters
        K = self.camera_intrinsics
        D = self.distortion_coeffs if self.distortion_coeffs is not None else np.zeros(5)

        # 4. NATIVE DRAWING
        # This function draws X(R), Y(G), Z(B) automatically
        # thickness=2 with LINE_AA is the secret for thin but not "jagged" lines
        cv2.drawFrameAxes(cv_image, K, D, rvec, tvec, axis_length, thickness=2)

        return cv_image