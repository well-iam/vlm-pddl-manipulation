import time
import re
import math
from scipy.spatial.transform import Rotation as R
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from .vision_system import VisionSystem

# Import fundamental ROS messages
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import (
    PlanningScene,
    PlanningSceneWorld,
    PlanningSceneComponents,
    CollisionObject,
    AttachedCollisionObject
)
from moveit_msgs.srv import GetPlanningScene
from std_srvs.srv import Empty
from perception_server.srv import GetSceneObjects

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException, \
    TransformException
import tf2_geometry_msgs  # To transform poses if needed

FINGER_HEIGHT = 0.015


class FrankaPerception:
    def __init__(self, node: Node, base_frame, ee_frame, hand_group_name):
        self.node = node
        self.logger = self.node.get_logger().get_child('perception')
        self.logger.debug("--- Initialization of FrankaPerception component ---")

        self.base_frame = base_frame
        self.ee_frame = ee_frame
        self.hand_group_name = hand_group_name

        self._initial_joint_state = None
        self._scene_initialized = False

        # TF Listener (To know current pose)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.node)

        self.vision = VisionSystem(self.node)

        # Temporary subscriber to read initial state
        self._joint_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self._initial_joint_state_callback,
            10
        )

        # Specific publisher for attach/detach
        self._attached_object_pub = self.node.create_publisher(
            AttachedCollisionObject,
            '/attached_collision_object',
            10
        )

        # Client for perception_server
        self._perception_cli = self.node.create_client(
            GetSceneObjects,
            '/get_scene_objects')

        # Publisher for Planning Scene
        # Needed to add obstacles (table) and prevent ghost or real collisions
        self._scene_pub = self.node.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        # Publisher for Planning Scene
        # Needed to add obstacles (table) and prevent ghost or real collisions
        self._co_pub = self.node.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )

        # Client to QUERY the scene (not to modify it)
        self._scene_client = self.node.create_client(
            GetPlanningScene,
            '/get_planning_scene'
        )

        self._clear_octomap_client = self.node.create_client(
            Empty,
            '/clear_octomap'
        )

        # TODO: Environment Initialization
        # it is necessary to define collision environment at startup
        # self._add_table_collision()
        self.add_target_object("franka_left", [0.72, 0.25, 0.35], [0.02, 0.6, 0.8])

    def get_initial_joint_state(self):
        return self._initial_joint_state

    def clear_octomap(self):
        """
        Empties current Octomap.

        :param self: Description
        """

        if not self._clear_octomap_client.wait_for_service(timeout_sec=1.0):
            self.logger.warn(f"Service /clear_octomap not available. Unable to reset Octomap.")
            return False

        req = Empty.Request()
        # Async call
        future = self._clear_octomap_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        return True

    def _initial_joint_state_callback(self, msg: JointState):
        if self._initial_joint_state is not None:
            return

        joint_state = []

        # Filter joints
        for i in range(0, 7):
            joint_state.append(msg.position[i])

        if len(joint_state) == 7:
            self._initial_joint_state = joint_state
            self.logger.debug("Home position saved (7 joints, gripper excluded)")
        else:
            self.logger.warn(f"Found {len(joint_state)} joints instead of 7. Retrying...")

    def scan_scene_world(self):
        """
        Calls perception and updates MoveIt

        :param self: Description
        """
        if not self._perception_cli.wait_for_service(timeout_sec=2.0):
            self.logger.error("Service perception_server offline!")
            return None

        # 1. Call service [HARD-CODED VALUES]
        req = GetSceneObjects.Request()
        req.target_shape = "box"
        req.target_dimensions = [0.05, 0.05, 0.05]

        # Plane caching logic
        if not self._scene_initialized:
            self.logger.debug("First scan: Looking for table...")
            req.recompute_support_surface = True
        else:
            self.logger.info("Rapid scan: Using cached table...")
            req.recompute_support_surface = False

        future = self._perception_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        res = future.result()

        if not res.scene_world.collision_objects:
            self.logger.warn("No objects found.")
            return None
        self._scene_initialized = True
        # 2. Prepare message for MoveIt
        scene_msg = PlanningScene()
        scene_msg.is_diff = True  # Add, do not delete everything else

        # Add found objects (Table + Cylinder)
        for co in res.scene_world.collision_objects:
            if co.pose.position.z > 0.1:
                continue
            co.operation = co.ADD
            scene_msg.world.collision_objects.append(co)
            self.logger.debug(f"Sending to MoveIt: {co.id} ({co.header.frame_id})")

        # 3. Publish
        self._scene_pub.publish(scene_msg)
        self.clear_octomap()
        return res

    def set_object_attached(self, object_name, attach=True):
        """
        Attaches (or detaches) a collision object to robot gripper.
        This disables collisions between gripper and object and makes object move with robot.

        :param self: Description
        :param object_name: Description
        :param attached: Description
        """
        msg = AttachedCollisionObject()
        msg.link_name = self.hand_group_name
        msg.object.id = object_name
        msg.touch_links = ['fr3_leftfinger', 'fr3_rightfinger']

        if attach:
            msg.object.operation = msg.object.ADD
            self.logger.debug(f"Attaching object '{object_name}' to hand...")
        else:
            msg.object.operation = msg.object.REMOVE
            self.logger.debug(f"Detaching object '{object_name}' from hand...")

        # Publish message
        self._attached_object_pub.publish(msg)

        # IMPORTANT: Give MoveIt time to update collision matrix
        # Without this sleep, subsequent planning might fail because
        # MoveIt still believes there is a collision.
        time.sleep(1.0)  # 0.5s - 1.0s is usually sufficient

    def _wait_for_object(self, object_name, timeout=Duration(seconds=2.0)):
        """
        Queries MoveIt until object appears in its memory.
        Ensures planner sees it before starting to plan.

        :param self: Description
        :param object_name: Description
        :param timeout: Description
        """

        # 1. Wait for service
        if not self._scene_client.wait_for_service(timeout_sec=1.0):
            self.logger.warn("Service /get_planning_scene not available.")
            return False

        req = GetPlanningScene.Request()
        # Ask only for Collision Objects to save bandwidth
        req.components.components = req.components.WORLD_OBJECT_NAMES

        start_time = self.node.get_clock().now()
        while (self.node.get_clock().now() - start_time) < timeout:
            # 2. Call service (Sync)
            future = self._scene_client.call_async(req)

            # Wait briefly for response
            while not future.done():
                time.sleep(0.01)
                if (self.node.get_clock().now() - start_time) > timeout:
                    return False

            response = future.result()

            # 3. Search name in known objects list
            # List is in response.scene.world.collision_objects
            known_objects = [obj.id for obj in response.scene.world.collision_objects]

            if object_name in known_objects:
                self.logger.info(f"Object '{object_name}' confirmed in Planning Scene.")
                return True

            time.sleep(0.1)  # Retry shortly

        self.logger.warn(f"Timeout: Object '{object_name}' did not appear in MoveIt.")
        return False

    def _add_table_collision(self):
        """
        Adds a virtual table to planning scene.
        Without this, real robot might attempt to plan trajectories that
        pass through the support surface.

        :param self: Description
        """
        table = CollisionObject()
        table.header.frame_id = self.base_frame
        table.id = "mounting table"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [1.0, 0.60, 0.1]  # Table dimensions taken manually (up to camera)

        pose = Pose()
        pose.position.x = 0.25 + 0.25 / 2  # Forward relative to robot
        pose.position.y = 0.20  # Left relative to robot
        pose.position.z = -0.07  # Below relative to robot base (z=0)
        pose.orientation.w = 1.0

        table.primitives.append(primitive)
        table.primitive_poses.append(pose)
        table.operation = CollisionObject.ADD

        # Publish multiple times for safety
        for _ in range(3):
            self._co_pub.publish(table)
            time.sleep(0.1)

        # Ensure MoveIt receives CollisionObject
        if self._wait_for_object(table.id):
            self.logger.debug("Collision Object 'Table' added to scene.")
            return True
        else:
            self.logger.debug("Collision Object 'Table' not added to scene")
            return False

    def add_target_object(self, object_name, position, size=[0.05, 0.05, 0.05]):
        """
        Adds object to manipulate to the scene.

        :param self: Description
        :param object_name: Description
        :param position: Description
        :param size: Description
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.base_frame
        collision_object.id = object_name

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size

        # TODO: Adjust orientation
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        # Publish multiple times for safety
        for _ in range(3):
            self._co_pub.publish(collision_object)
            time.sleep(0.1)

        # Ensure MoveIt receives CollisionObject
        if self._wait_for_object(object_name):
            self.logger.debug(f"Added object '{object_name}' to MoveIt scene.")
            return True
        else:
            self.logger.debug(f"Collision Object '{object_name}' not added to scene")
            return False

    def get_current_pose(self):
        """
        Reads current end effector pose relative to base
        Returns: PoseStamped or None in case of error

        :param self: Description
        """
        try:
            # Search most recent available transform
            trans = self._tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),  # "Give me latest valid data available" (for old static frames)
                timeout=rclpy.duration.Duration(seconds=1.0)  # "If buffer empty, wait a bit for fill" (wait transform)
            )

            # Convert TransformStamped to PoseStamped for convenience
            current_pose = PoseStamped()
            current_pose.header.frame_id = self.base_frame
            current_pose.header.stamp = self.node.get_clock().now().to_msg()

            current_pose.pose.position.x = trans.transform.translation.x
            current_pose.pose.position.y = trans.transform.translation.y
            current_pose.pose.position.z = trans.transform.translation.z
            current_pose.pose.orientation = trans.transform.rotation

            return current_pose

        except Exception as e:
            self.logger.error(f"Unable to read current pose: {e}")
            return None

    def get_planning_scene(self):
        """
        Calls GetPlanningScene service.
        Returns: moveit_msgs.msg.PlanningScene or None
        """
        if not self._scene_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service /get_planning_scene not available")
            return None

        req = GetPlanningScene.Request()
        req.components.components = (
                PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        future = self._scene_client.call_async(req)

        # Sync Wait (blocking)
        rclpy.spin_until_future_complete(self.node, future)

        try:
            response = future.result()
            # Return scene directly, not whole response wrapper
            return response.scene
        except Exception as e:
            self.logger.error(f"Error calling planning scene service: {e}")
            return None

    def get_collision_objects(self):
        scene = self.get_planning_scene()
        if not scene:
            return []
        return scene.world.collision_objects

    def get_object_from_scene(self, object_name) -> tuple[PoseStamped, list]:
        """
        Retrieves PoseStamped and dimensions [x,y,z] of object.
        """
        scene = self.get_planning_scene()
        if not scene:
            return None, None

        target_obj = None
        # 1. Search in WORLD (CollisionObjects)
        for obj in scene.world.collision_objects:
            if obj.id == object_name:
                target_obj = obj
                break

        # 2. If not found, search in ROBOT (AttachedCollisionObjects)
        if not target_obj:
            for attached_obj in scene.robot_state.attached_collision_objects:
                if attached_obj.object.id == object_name:
                    target_obj = attached_obj.object
                    break

        if not target_obj:
            self.logger.error(f"Object '{object_name}' not found in MoveIt memory!")
            return None, None

        # Primitives validation
        if target_obj.pose is None:
            self.logger.error(f"Object '{object_name}' found but lacking 'pose' field.")
            return None, None

        # --- CREATE POSE STAMPED (Crucial for frames) ---
        obj_pose_stamped = PoseStamped()
        # Copy original header (contains frame_id, e.g., 'world' or 'fr3_link0')
        obj_pose_stamped.header = target_obj.header
        # Copy geometric pose
        obj_pose_stamped.pose = target_obj.pose

        # Dimensions
        primitive = target_obj.primitives[0]
        dims = primitive.dimensions  # Returns list [x, y, z]

        return obj_pose_stamped, dims

    def get_grasp_pose(self, object_name, params=None) -> PoseStamped:
        """
        Calculates PoseStamped to grasp the object.
        Applies Top-Down logic.
        Optimized to always grasp the SHORT SIDE.
        """

        # 1. Where is the object?
        obj_pose_stamped, dims = self.get_object_from_scene(object_name)
        if obj_pose_stamped is None:
            self.logger.warn("get_object_from_scene() failed")
            return None
        obj_pose = obj_pose_stamped.pose

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = self.base_frame
        grasp_pose.header.stamp = self.node.get_clock().now().to_msg()

        # 2. Position
        grasp_pose.pose.position = obj_pose.position
        # Z Logic
        box_height = dims[SolidPrimitive.BOX_Z]
        grasp_pose.pose.position.z += box_height / 2 + 0.01  # cm for RealSense noise

        # TODO: We assume object BBOX are resting on plane!
        # 3. Orientation
        obj_quat = [obj_pose.orientation.x, obj_pose.orientation.y,
                    obj_pose.orientation.z, obj_pose.orientation.w]
        r_obj = R.from_quat(obj_quat)
        yaw = r_obj.as_euler('zyx')[0]

        # ---------- SHORT AXIS CORRECTION LOGIC
        # Franka has fingers moving along its Y axis.
        # Basically, object 'yaw' aligns X_gripper with X_object and Y_gripper with Y_object.
        obj_size_x = dims[SolidPrimitive.BOX_X]
        obj_size_y = dims[SolidPrimitive.BOX_Y]

        # If Y_object > X_object, means we are trying to grasp long side.
        # We must rotate 90 degrees to align Y_gripper with X_object (which is short side).
        if obj_size_y > obj_size_x:
            self.logger.debug(f"Side Y ({obj_size_y:.3f}) > Side X ({obj_size_x:.3f}). Rotating 90°.")
            yaw += math.pi / 2
        else:
            self.logger.debug(f"Side X ({obj_size_x:.3f}) >= Side Y ({obj_size_y:.3f}). Keeping orientation.")

        # Construct gripper rotation
        # We want:
        #   a. rotate around Z by 'yaw' (to align with object)
        #   b. rotate around X by 180° (to point down)
        #   Watch out for order! (First I turn towards object, then look down)

        # Rotation pointing down (Standard Franka)
        r_down = R.from_euler('x', 180, degrees=True)

        # Object alignment rotation
        r_align = R.from_euler('z', yaw, degrees=False)

        # Combination: R_final = R_align * R_down
        # Note: Scipy multiplies from left
        r_final = r_align * r_down

        # Convert to quaternion
        final_quat = r_final.as_quat()
        grasp_pose.pose.orientation.x = final_quat[0]
        grasp_pose.pose.orientation.y = final_quat[1]
        grasp_pose.pose.orientation.z = final_quat[2]
        grasp_pose.pose.orientation.w = final_quat[3]

        # Apply Gemini OFFSET (If present)
        if params and "grasp_offset_local" in params:
            offsets = params["grasp_offset_local"]
            if len(offsets) == 3:
                dx, dy, dz = offsets[0], offsets[1], offsets[2]
                self.logger.info(f"Applying Gemini Offset (Local): [{dx}, {dy}, {dz}]")
                # Rotate offset vector (which is local to object) into Robot frame
                # offset_world = R_object * offset_local
                offset_world = r_obj.apply([dx, dy, dz])

                # Add to current target position
                grasp_pose.pose.position.x += offset_world[0]
                grasp_pose.pose.position.y += offset_world[1]
                grasp_pose.pose.position.z += offset_world[2]

            else:
                self.logger.warn("Parameter 'grasp_offset_local' invalid (requires 3 values)")

        # self.logger.info(f"Grasp Pose calculated for '{object_name}': Z={grasp_pose.pose.position.z:.3f}, Yaw={np.degrees(yaw):.1f} ")
        return grasp_pose

    def get_place_pose(self, object_name, target_name) -> PoseStamped:
        """
        Calculates PoseStamped to Place object 'object_name' OVER 'target_name'.
        Logic:
          - X, Y: Target center.
          - Z: Sum of half-heights (Stacking) + small buffer.
          - Orientation: Aligns gripper to target orientation (to stack neatly).
        """

        # 1. Retrieve info on Target (where to place)
        target_pose_stamped, target_dims = self.get_object_from_scene(target_name)
        if target_pose_stamped is None:
            self.logger.error(f"Target '{target_name}' not found in scene.")
            return None
        target_pose = target_pose_stamped.pose
        self.logger.debug(f"Pose: '{target_pose}'. Dims: {target_dims}.")

        # 2. Retrieve info on Object we are holding (needed for its height)
        # Note: assume object_name is still tracked/known to scene
        obj_pose, obj_dims = self.get_object_from_scene(object_name)
        self.logger.debug(f"Pose: '{obj_pose}'. Dims: {obj_dims}.")
        if obj_pose is None:
            self.logger.error(f"Object '{object_name}' (in hand) not found/lost.")
            return None

        place_pose = PoseStamped()
        place_pose.header.frame_id = self.base_frame
        place_pose.header.stamp = self.node.get_clock().now().to_msg()

        # 3. Position (Stacking Logic)
        # X and Y coincide with target center
        place_pose.pose.position.x = target_pose.position.x
        place_pose.pose.position.y = target_pose.position.y

        # Z Logic:
        # Z_target (center) + Target Half-Height = Target Top Surface
        # Target Top Surface + Object Half-Height = New Object Center
        target_half_h = target_dims[SolidPrimitive.BOX_Z] / 2
        obj_half_h = obj_dims[SolidPrimitive.BOX_Z] / 2

        # Add safety buffer (e.g. 2mm) to not crush target before opening gripper
        release_buffer = 0.010

        place_pose.pose.position.z = target_pose.position.z + target_half_h + obj_half_h * 2 + release_buffer

        # 4. Orientation
        # We want object aligned to target (e.g. stacking two aligned boxes).
        # Extract target Yaw.
        target_quat = [target_pose.orientation.x, target_pose.orientation.y,
                       target_pose.orientation.z, target_pose.orientation.w]

        r_target = R.from_quat(target_quat)
        target_yaw = r_target.as_euler('zyx')[0]

        self.logger.debug(f"Place alignment to target '{target_name}' with Yaw: {math.degrees(target_yaw):.1f}°")

        # Construct gripper rotation for Place
        # a. Rotate around Z like target (target_yaw)
        # b. Rotate around X by 180° (Top-Down standard Franka)

        # Rotation pointing down
        r_down = R.from_euler('x', 180, degrees=True)

        # Alignment rotation to target
        r_align = R.from_euler('z', target_yaw, degrees=False)

        # Combination: R_final = R_align * R_down
        r_final = r_align * r_down

        # Convert to quaternion
        final_quat = r_final.as_quat()
        place_pose.pose.orientation.x = final_quat[0]
        place_pose.pose.orientation.y = final_quat[1]
        place_pose.pose.orientation.z = final_quat[2]
        place_pose.pose.orientation.w = final_quat[3]

        # Debug log
        # self.logger.info(f"Place Pose calculated over '{target_name}': Z={place_pose.pose.position.z:.3f}")

        return place_pose

    def transform_pose(self, input_pose, source_frame, target_frame):
        """
        Transforms a geometry_msgs/Pose from source frame to target frame.

        :param input_pose: The geometry_msgs/Pose to transform
        :param source_frame: String, frame in which input_pose is expressed (e.g. 'fr3_link0')
        :param target_frame: String, frame where you want result (e.g. 'camera_color_optical_frame')
        :return: geometry_msgs/Pose transformed, or None if failed.
        """

        if source_frame == target_frame:
            return input_pose

        try:
            # 2. Search transform in buffer
            # "Give me transform TO go to target_frame STARTING FROM source_frame"
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),  # TimePointZero: take latest available transform
                timeout=Duration(seconds=1.0)  # Wait max 1 second if TF has not arrived yet
            )

            # 3. Apply mathematical transform
            transformed_pose = tf2_geometry_msgs.do_transform_pose(input_pose, transform)

            return transformed_pose

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.logger.error(f"TF Error transforming from {source_frame} to {target_frame}: {e}")
            return None

    def sanitize_id(self, name):
        """Transforms 'Sugar Box!' into 'sugar_box' for ROS safety"""
        # Removes non-alphanumeric characters (except spaces)
        name = re.sub(r'[^a-zA-Z0-9\s]', '', name)
        # Replaces spaces with underscore and converts to lowercase
        return name.strip().replace(' ', '_').lower()