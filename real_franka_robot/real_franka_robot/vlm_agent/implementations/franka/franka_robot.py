import time
import rclpy
from rclpy.node import Node
# Import fundamental ROS messages
from geometry_msgs.msg import PoseStamped

from .components.gripper import FrankaGripper
from .components.planning import FrankaPlanning
from .components.perception import FrankaPerception
from .components.actuation import FrankaActuation
from .components.vision_system import VisionSystem  # Used for hinting


class FrankaRobot(Node):
    def __init__(self, node_name="real_franka_robot_node"):
        super().__init__(node_name)
        self.logger = self.get_logger()
        self.logger.debug("--- Initialization of RealFrankaRobot interface ---")

        self.arm_group_name = "fr3_arm"
        self.hand_group_name = "fr3_hand"
        self.joint_names = [f"fr3_joint{i}" for i in range(1, 8)]
        self.finger_links = ['fr3_leftfinger', 'fr3_rightfinger']

        # Reference Frames
        self.base_frame = "fr3_link0"
        self.ee_frame = "fr3_hand_tcp"

        # Table name (as in domain.pddl and perception_server_params.yaml)
        self.support_surface_id = "table"

        # --- COMPONENT INITIALIZATION
        self.gripper = FrankaGripper(self)
        self.planning = FrankaPlanning(self, self.arm_group_name, self.ee_frame, self.joint_names)
        self.perception = FrankaPerception(self, self.base_frame, self.ee_frame, self.hand_group_name)
        self.actuation = FrankaActuation(self)

        # Gripper Memory (Since camera is blind to gripper)
        self.gripper_state = {
            'is_holding': False,
            'held_object_name': 'unnamed',
        }

        # Skill Name Map -> Python Functions
        # This dictionary connects the name Gemini chooses to the function to execute.
        self.available_skills = {
            "pick": self.perform_pick,
            "place": self.perform_place,
            "done": self.perform_done
        }
        self.available_detectors = {
            "pick": self.pick_success_detector,
            "place": self.place_success_detector,
        }

        # Stacking/Displacement Logic
        self.obj_counter = 0
        disp = 0.04
        self.place_disp = [[-disp, -disp], [disp, disp], [disp, -disp], [-disp, disp]]
        self.logger.info("FrankaRobot ready and operational.")

    def go_home(self):
        initial_joint_state = self.perception.get_initial_joint_state()
        if initial_joint_state is None:
            self.logger.error("No Home position saved!")
            return False

        self.logger.debug("Return to base (Homing)...")
        planned_trajectory = self.planning.plan_trajectory_to_config(initial_joint_state)
        if not planned_trajectory:
            return False

        # 4. Execute
        return self.actuation.execute_trajectory(planned_trajectory)

    def move_to_relative_pose(self, dx=0.0, dy=0.0, dz=0.0, ignore_collisions=False, threshold=1.0):
        """
        Moves the robot by a Cartesian delta relative to current position. Maintains current orientation.
        """
        self.logger.debug(f"Requested relative movement: [{dx}, {dy}, {dz}]")

        # 1. Get current pose
        current_pose = self.perception.get_current_pose()
        if current_pose is None:
            return False

        # 2. Calculate new target
        target_pose = PoseStamped()
        target_pose.header = current_pose.header  # Copy Frame and Timestamp

        target_pose.pose.position.x = current_pose.pose.position.x + dx
        target_pose.pose.position.y = current_pose.pose.position.y + dy
        target_pose.pose.position.z = current_pose.pose.position.z + dz

        # Maintain original orientation (or change it)
        target_pose.pose.orientation = current_pose.pose.orientation

        # 3. Plan
        planned_trajectory = self.planning.plan_linear_path(target_pose, ignore_collisions, threshold)
        if not planned_trajectory:
            return False

        # 4. Execute
        return self.actuation.execute_trajectory(planned_trajectory)

    def move_to_pose(self, target_pose: PoseStamped):
        self.logger.debug(
            f"Requested movement to position: [{target_pose.pose.position.x}, {target_pose.pose.position.y}, {target_pose.pose.position.z}]")

        # Plan
        planned_trajectory = self.planning.plan_trajectory(target_pose)
        if not planned_trajectory:
            return False

        # Execute
        return self.actuation.execute_trajectory(planned_trajectory)

    def perform_pick(self, object_name, params=None):
        """
        """

        actuator = self.actuation
        planner = self.planning
        perception = self.perception
        gripper = self.gripper

        self.logger.info(f"Starting PICK procedure for '{object_name}'...")
        approach_height = 0.08

        # Calculate Grasp Pose
        grasp_pose = perception.get_grasp_pose(object_name, params)
        if not grasp_pose:
            return False
        self.logger.debug(f"Obtained grasp pose: '{grasp_pose.pose}'...")

        # Pre-Grasp
        pre_grasp = PoseStamped()
        pre_grasp.header = grasp_pose.header
        pre_grasp.pose.position.x = grasp_pose.pose.position.x
        pre_grasp.pose.position.y = grasp_pose.pose.position.y
        pre_grasp.pose.position.z = grasp_pose.pose.position.z + approach_height
        pre_grasp.pose.orientation = grasp_pose.pose.orientation

        path_to_pre_grasp = planner.plan_trajectory(pre_grasp)
        if not actuator.execute_trajectory(path_to_pre_grasp):
            return False

        # Open Gripper
        gripper.open()

        # 4. Descent
        if not self.move_to_relative_pose(dz=-approach_height, ignore_collisions=True):
            return False

        # 5. Close Gripper
        self.logger.debug("Grasping...")
        grasp_success = gripper.grasp(width=0.05, force=0.1)
        if not grasp_success:
            # TODO: Emergency ascent? Or stop?
            self.gripper_state['is_holding'] = False
            self.gripper_state['held_object_name'] = "unnamed"
            self.move_to_relative_pose(dz=+approach_height, ignore_collisions=True)
            self.go_home()
            return False
        self.gripper_state['is_holding'] = True
        self.gripper_state['held_object_name'] = object_name
        # Attach Software
        perception.set_object_attached(object_name, attach=True)

        # 6. Ascent
        self.logger.debug("Lifting...")
        if not self.move_to_relative_pose(dz=+approach_height, ignore_collisions=True):
            return False

        # 7. Return to rest position
        # self.logger.debug("RETURN TO INITIAL POSITION...")
        # self.go_home()

        return True

    def perform_place(self, object_name, target_name):
        planner = self.planning
        perception = self.perception
        actuator = self.actuation
        gripper = self.gripper
        approach_height = 0.1

        # 2. Definition of target Cartesian poses relative to robot base
        place_pose = self.perception.get_place_pose(object_name, target_name)
        if not place_pose:
            self.logger.error("Unable to calculate place pose.")
            return False

        # 2. MOVEMENT STRATEGY (Relative Pose Calculation)
        # Here we apply safety logic you wanted to handle in caller

        # Stacking/Displacement DEMO
        dx, dy = self.place_disp[self.obj_counter]
        self.obj_counter += 1
        # Pre-Place
        pre_place = PoseStamped()
        pre_place.header = place_pose.header
        pre_place.pose.position.x = place_pose.pose.position.x + dx  # Stacking/Displacement DEMO
        pre_place.pose.position.y = place_pose.pose.position.y + dy  # Stacking/Displacement DEMO
        pre_place.pose.position.z = place_pose.pose.position.z + approach_height
        pre_place.pose.orientation = place_pose.pose.orientation

        self.logger.debug(f"PLACE_POSE: {place_pose.pose.position}")
        self.logger.debug(f"PRE_PLACE_POSE: {pre_place.pose.position}")

        # 3. Movement towards pre-place pose
        path_to_pre_place = planner.plan_trajectory(pre_place)
        if not actuator.execute_trajectory(path_to_pre_place):
            return False

        # 4. Final approach movement (linear, assumed safe)
        if not self.move_to_relative_pose(dz=-approach_height, ignore_collisions=True):
            return False

        # 5. Object release
        gripper.open()
        # Detach Software
        perception.set_object_attached(object_name, attach=False)
        self.gripper_state['is_holding'] = False
        self.gripper_state['held_object_name'] = "unnamed"

        # 6. Ascent
        self.logger.debug("Lifting...")
        if not self.move_to_relative_pose(dz=+approach_height, ignore_collisions=True):
            return False

        # 7. Return to rest position
        self.logger.debug("RETURN TO INITIAL POSITION...")
        self.go_home()
        return True

    def perform_done(self):
        print("🤖 FAKE_EXECUTE: done (task finished)")
        return True

    def pick_success_detector(self, object_name: str) -> [bool, str]:
        """
        Critical function for Sim-to-Real.
        Checks if memory matches reality.
        If gripper is closed (< 5mm) but memory says we are holding something,
        it means the object fell.
        """
        holding_object = self.gripper_state["is_holding"] and (self.gripper_state["held_object_name"] == object_name)
        if holding_object:
            return True, "True"
        return False, "Object not grasped"

    # TODO: implement (take snapshot and ask VLM if object correctly placed)
    def place_success_detector(self, object_name, target_name) -> [bool, str]:
        return True, "True"

    def get_annotated_image_and_map(self):
        perception = self.perception
        vision = perception.vision

        # Obtain objects from MoveIt
        current_collision_objects = self.perception.get_collision_objects()
        scene_is_empty = len(current_collision_objects) <= 1  # I put a collision object Franka_left

        collision_objects = []
        rgb_image = None

        if scene_is_empty:
            # 1. Scene scan: gets 3D objects and raw RGB image
            self.logger.info("Scanning scene from scratch.")
            res = perception.scan_scene_world()
            if not res:
                return None, None

            # collision_objects = res.scene_world.collision_objects
            # rgb_image = res.rgb_image

        # self.logger.info("Retrieving Objects from MoveIt Memory")
        collision_objects = self.perception.get_collision_objects()
        rgb_image = self.perception.vision.take_snapshot()
        if rgb_image is None:
            self.logger.error("Unable to retrieve image from camera!")
            return None, None
        # ROS Image -> OpenCV Image
        cv_image = vision.bridge.imgmsg_to_cv2(rgb_image, desired_encoding="bgr8")

        # Map to translate: Visual ID (for Gemini) -> Real ID (for Robot)
        # Ex: {'1': 'box_0', '2': 'box_5'}
        visual_id_to_moveit_map = {}
        # Dictionary to annotate on image
        points_text_map = {}

        # Counter to generate sequential visual IDs (1, 2, 3...)
        visual_counter = 1
        for co in collision_objects:
            if co.id == self.support_surface_id or co.id == "franka_left":
                # If it's the table, ignore for annotation
                pass
            else:
                # CAREFUL that sometimes you need to take 'pose' field, other times 'primitive_poses'
                co_primitive_pose = co.primitive_poses[0]
                co_pose = co.pose
                self.logger.debug(f"co_primitive_pose: {co_primitive_pose.position}")
                self.logger.debug(f"co_pose: {co_pose.position}")
                self.logger.debug(
                    f"Projecting object coordinates {co.id} (relative to {co.header.frame_id}): {co_pose.position.x}, {co_pose.position.y}, {co_pose.position.z}")
                co_pose_camera = self.perception.transform_pose(co_pose, co.header.frame_id,
                                                                self.perception.vision.color_optical_frame)
                # 1. Projection 3D -> 2D
                uv_coords = vision.project_3d_to_2d(co_pose_camera.position)
                self.logger.debug(f"Coordinates (u,v): {uv_coords}")

                cv_image = self.perception.vision.draw_local_axes(cv_image, co_pose_camera)

                if uv_coords is not None:
                    u, v = int(uv_coords[0]), int(uv_coords[1])

                    # 2. Create simple visual ID (numeric string)
                    visual_id = str(visual_counter)
                    visual_counter += 1

                    # 3. Annotate image with visual ID
                    points_text_map[(u, v)] = visual_id

                    # 4. Save correspondence for later
                    visual_id_to_moveit_map[visual_id] = co.id

                    # Here we annotate image with "1", "2", "3"... very clean
        cv_annotated_img = self.perception.vision.annotate_image_with_points(cv_image, points_text_map)

        # Not needed, because to maintain chat_history it is more efficient to serialize cv_img
        """ # OpenCV Image -> PIL Image (for sending to Gemini)
        pil_annotated_image = vision.cv_to_pil(cv_annotated_img) """

        # Return image AND map to translate Gemini's response
        return cv_annotated_img, visual_id_to_moveit_map

    def shutdown(self):
        if self.gripper_state["is_holding"]:
            self.perception.set_object_attached(self.gripper_state["held_object_name"], attach=False)
        self.gripper.open()
        time.sleep(1.0)
        self.go_home()