import logging
from tesi_gemini_robotics.interfaces.robot_interface import RobotInterface
from .components.perception import CoppeliaPerception
from .components.planning import CoppeliaPlanner
from .components.actuation import CoppeliaActuator
from .components.gripper import CoppeliaGripper

logger = logging.getLogger(__name__)

class CoppeliaSimRobot(RobotInterface):
    """
    Class that encapsulates the connection and control of a robot in CoppeliaSim.

    Manages handles, IK/OMPL environment and provides high-level methods
    to execute robot skills (e.g., pick).
    """
    def __init__(self, client, sim, simIK, simOMPL, robot_name='Franka'):
        """
        Initializes the robot controller.

        Args:
            client: Main ZMQ client object.
            sim: 'sim' API client object.
            simIK: 'simIK' API client object.
            simOMPL: 'simOMPL' API client object.
            robot_name (str): Name of the robot in CoppeliaSim scene.

        Raises:
            Exception: If initial setup fails (e.g. handles not found).
        """

        logger.info("--- Initializing RobotController ---")
        self.client, self.sim, self.simIK, self.simOMPL = client, sim, simIK, simOMPL
        self.robot_name = robot_name

        # 1. Retrieve Scene Handles
        self.handles = self._setup_scene_handles()
        if not self.handles:
            raise RuntimeError("Setup failed: impossible to retrieve scene handles.")

        self.perception = CoppeliaPerception(self.sim, self.handles)
        self.planner = CoppeliaPlanner(self.sim, self.simIK, self.simOMPL, self.handles)
        self.actuator = CoppeliaActuator(self.client, self.sim, self.handles)
        self.gripper = CoppeliaGripper(self.sim, self.handles)


        # Map Skill Names -> Python Functions
        # This dictionary links the name Gemini will choose to the function to execute.
        self.available_skills = {
            "pick": self.perform_pick,
            "place": self.perform_place,
            "nudge": self.perform_nudge,
            "done": self.perform_done
        }
        self.available_detectors = {
            "pick": self.pick_success_detector,
            "place": self.place_success_detector,
            "nudge": self.nudge_success_detector,
        }
        logger.debug(f"✅ Available robot skills: {list(self.available_skills.keys())}")
        logger.info("✅ RobotController initialized successfully.")

    def _setup_scene_handles(self):
        """
        Obtains and returns handles for key robot and scene components
        robustly, searching objects by type and hierarchy.
        """
        sim = self.sim
        logger.debug("Retrieving handles of objects in scene...")
        handles = {}

        try:
            # 1. OBTAIN MAIN CONTROL OBJECTS
            # These are the few names reasonable to keep fixed, as they
            # represent main models and control elements.
            handles['base'] = sim.getObject('/Franka')
            handles['fake_base'] = sim.getObject('/FakeFranka')
            handles['gripper'] = sim.getObject('/Franka/FrankaGripper')
            handles['target'] = sim.getObject('/Franka/Franka_target')
            handles['tip'] = sim.getObject('/Franka/Franka_tip')  # Search connection point (end-effector)

            # 2. FIND JOINTS DYNAMICALLY (KEY PART)
            # Instead of searching 'joint1', 'joint2', etc., we ask simulator to give us
            # all "joint" type objects that are children of Franka model.
            all_robot_joints = sim.getObjectsInTree(handles['base'], sim.object_joint_type, 0)

            # Robotic arm joints
            joint_names = [
                f"  - Name: '{sim.getObjectAlias(h)}', Handle: {h}"
                for h in all_robot_joints
            ]
            joint_list_string = "\n".join(joint_names)
            logger.debug(f"""
            Found {len(all_robot_joints)} 'joint' type objects total under '/Franka':
            {joint_list_string}""")

            # Gripper joints
            gripper_joints_handles = sim.getObjectsInTree(handles['gripper'], sim.object_joint_type, 0)
            arm_joints_handles = [h for h in all_robot_joints if h not in gripper_joints_handles]

            # Fake robotic arm joints
            fake_robot_joints = sim.getObjectsInTree(handles['fake_base'], sim.object_joint_type, 0)

            handles['arm_joints'] = arm_joints_handles
            handles['gripper_joints'] = gripper_joints_handles
            handles['fake_arm_joints'] = fake_robot_joints
            logger.debug(f"""
            --- Final Lists Composition ---
              ARM Joints ({len(handles['arm_joints'])}): {handles['arm_joints']}
            ------------------------------------
            """)

            # 3. OBTAIN COLLECTIONS
            # List of objects composing your environment
            all_top_level_shape_handles = sim.getObjectsInTree(sim.handle_scene, sim.sceneobject_shape, 2)
            ignore_list = [handles['base'], handles['fake_base']]#, sim.getObject('/Floor')]
            env_objects_handles = [h for h in all_top_level_shape_handles if h not in ignore_list]
            logger.debug(all_top_level_shape_handles)
            logger.debug(env_objects_handles)

            # Call new function to create collections
            collection_handles = self._setup_collections_via_api(env_objects_handles)
            if not collection_handles:
                logger.critical("Impossible to create collections, terminating program.")
                return None

            # For collections, the only way is to use their name.
            handles['robot_collection'] = collection_handles[0]
            handles['fake_robot_collection'] = collection_handles[1]
            handles['environment_collection'] = collection_handles[2]

            # Summary for verification
            logger.debug(f"  - Found robot: '{sim.getObjectAlias(handles['base'])}'")
            logger.debug(f"  - Found {len(handles['arm_joints'])} arm joints.")
            logger.debug(f"  - Found {len(handles['gripper_joints'])} gripper joints.")
            logger.debug(f"  - Found IK target: '{sim.getObjectAlias(handles['target'])}'")

        except Exception as e:
            logger.exception(f"❌ ERROR during handle retrieval: ")
            logger.error(
                "    Check that all main objects (e.g. /Franka, /FrankaGripper, /Franka_target, /Cuboid) "
                "are present and have correct names in scene.")
            return None

        logger.info("✅ Handles retrieved successfully.")
        return handles

    #TODO
    def _setup_collections_via_api(self, environment_object_handles):
        """
        Creates and populates 'Robot' and 'Environment' collections dynamically via API.
        This is the modern and correct method for recent CoppeliaSim versions.
        """
        logger.info("Dynamic collection setup via API...")

        sim = self.sim
        robot_name = self.robot_name
        try:
            # --- 1. CREATE COLLECTIONS (or find if exist) ---
            robot_collection_handle = sim.createCollection(0)
            logger.info("  - 'Robot' collection created dynamically.")

            fake_robot_collection_handle = sim.createCollection(0)

            env_collection_handle = sim.createCollection(0)
            logger.info("  - 'Environment' collection created dynamically.")

            # --- 2. POPULATE 'ROBOT' COLLECTION ---
            # We add entire robot model (hierarchy tree) to collection.
            try:
                robot_handle = sim.getObject(f'/{robot_name}')
                # sim.handle_tree says include object and all its children.
                # options=0 means "add".
                # options=2 means exclude tree base
                sim.addItemToCollection(robot_collection_handle, sim.handle_tree, robot_handle, 2)
                logger.info(f"  - Added entire tree of robot '{robot_name}' to 'Robot' collection.")
                logger.debug(f'ROBOT COLLECTION HANDLES: {sim.getCollectionObjects(robot_collection_handle)}')
            except Exception as e:
                logger.exception(f"    - ERROR: Impossible to find or add robot '{robot_name}': ")

            try:
                fake_robot_handle = sim.getObject('/FakeFranka')
                sim.addItemToCollection(fake_robot_collection_handle, sim.handle_tree, fake_robot_handle, 2)
                logger.info(f"  - Added entire tree of robot 'Fake{robot_name}' to 'FakeRobot' collection.")
                logger.debug(
                    f'FAKE ROBOT COLLECTION HANDLES: {sim.getCollectionObjects(fake_robot_collection_handle)}')
            except Exception as e:
                logger.exception(f"    - ERROR: Impossible to find or add robot '{robot_name}': ")

            # --- 3. POPULATE 'ENVIRONMENT' COLLECTION ---
            for object_handle in environment_object_handles:
                try:
                    # sim.handle_single says add only this specific object.
                    sim.addItemToCollection(env_collection_handle, sim.handle_single, object_handle, 0)
                except Exception as e:
                    logger.exception(
                        f"    - Warning: Environment object '{sim.getObjectAlias(object_handle)}' not found in scene.")

            debug_env_collection_handles = sim.getCollectionObjects(env_collection_handle)
            debug_env_collection_names = [sim.getObjectAlias(f'{h}') for h in debug_env_collection_handles]
            logger.debug(f'NAMES OF OBJECTS IN ENVIRONMENT COLLECTION: {debug_env_collection_names}')
            logger.info("✅ Collections created and populated successfully.")

            return [robot_collection_handle, fake_robot_collection_handle, env_collection_handle]

        except Exception as e:
            logger.exception(f"❌ Critical error during collection management: ")
            return None


    def perform_pick(self, object_name):
        client, sim, simIK, simOMPL = self.client, self.sim, self.simIK, self.simOMPL
        handles = self.handles
        planner = self.planner
        actuator = self.actuator
        gripper = self.gripper

        success = False
        object_handle = sim.getObject(f'/{object_name}')
        try:
            # 1. Initial rest position
            # home_q = [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]
            home_q = [sim.getJointPosition(h) for h in handles['arm_joints']]

            # 2. Definition of target Cartesian poses relative to robot base
            logger.debug('CALCULATING TARGET POSES')
            grasp_pose = self.perception.get_grasp_pose(object_name)
            if not grasp_pose:
                logger.error("Impossible to calculate grasp-pose.")
                return False

            # PRE-GRASP: 15 cm above bounding box
            pre_grasp_pose = list(grasp_pose)
            pre_grasp_pose[11] += 0.15

            # LIFT: Lift 20 cm relative to grasp
            lift_pose = list(grasp_pose)
            lift_pose[11] += 0.20

            # 3. Movement towards pre-grasp pose
            logger.debug('MOVING TOWARDS PRE-GRASP')
            pre_grasp_q = planner.find_valid_ik_solution(pre_grasp_pose)
            if not pre_grasp_q:
                logger.error("Impossible to calculate pre-grasp configuration.")
                return False

            path_to_pre_grasp = planner.plan_path_rrt(pre_grasp_q)
            if not path_to_pre_grasp:
                logger.error("Impossible to calculate path towards pre-grasp pose.")
                return False
            actuator.execute_path(path_to_pre_grasp)

            # 4. Final approach movement (linear, assumed safe)
            logger.debug('LOWERING VERTICALLY TOWARDS CUBE')
            # setBoolProperty() NEEDED TO MAKE OBJECT BECOME CHILD OF GRIPPER
            # print(sim.getBoolProperty(object_handle, 'dynamic'))
            # print(sim.getBoolProperty(object_handle, 'respondable'))
            sim.setBoolProperty(object_handle, 'dynamic', False, 0)
            # sim.setBoolProperty(object_handle, 'respondable', False, 0)

            path_to_grasp = planner.plan_linear_path(grasp_pose)
            if not path_to_grasp:
                logger.error("Impossible to calculate path towards grasp pose.")
                return False
            actuator.execute_path(path_to_grasp)
            gripper.open()

            # 5. Object grasping
            if gripper.detect():
                gripper.attach_object(object_name)
                gripper.close()

            # 6. Object lifting
            lift_q = planner.find_valid_ik_solution(lift_pose)
            if not lift_q:
                logger.error("Impossible to calculate post-grasp configuration.")
                return False
            path_to_lift = planner.plan_linear_path(lift_pose)
            if not path_to_grasp:
                logger.error("Impossible to calculate path towards post-grasp pose.")
                return False
            actuator.execute_path(path_to_lift)

            # 7. Holding position for 3 seconds
            logger.debug("Holding object in position...")
            # for _ in range(60):  # 60 steps * 50ms/step = 3 seconds
            #     client.step()
            # time.sleep(3)

            # 8. Return to rest position
            path_to_home = planner.plan_path_rrt(home_q)
            if not path_to_home:
                logger.error("Impossible to calculate path towards Home pose.")
                return False
            actuator.execute_path(path_to_home)
            success = True

        finally:
            if not success:
                logger.debug("Object state reset to 'dynamic'")
                sim.setBoolProperty(object_handle, 'dynamic', True, 0)

        return success

    def perform_place(self, object_name, target_name):
        client, sim, simIK, simOMPL = self.client, self.sim, self.simIK, self.simOMPL
        handles = self.handles
        motion = self.planner
        actuator = self.actuator
        gripper = self.gripper

        # 1. Initial rest position
        home_q = [sim.getJointPosition(h) for h in handles['arm_joints']]

        # 2. Definition of target Cartesian poses relative to robot base
        place_pose, obj_height = self.perception.get_place_pose(object_name, target_name)
        if not place_pose:
            logger.error("Impossible to calculate place pose.")
            return False

        # 2. MOVEMENT STRATEGY (Relative poses calculation)
        # Here we apply safety logic you wanted to manage in caller

        # Pre-Place: 10 cm + half object height above destination
        pre_place_pose = list(place_pose)
        pre_place_pose[11] += (obj_height / 2.0) + 0.10

        # Post-Place (Lift): Safety rise (e.g. 20 cm above)
        lift_pose = list(place_pose)
        lift_pose[11] += 0.20

        logger.debug(f"PLACE_POSE: {place_pose}")
        logger.debug(f"PRE_PLACE_POSE: {pre_place_pose}")
        logger.debug(f"LIFT_POSE: {lift_pose}")

        # 3. Movement towards pre-grasp pose
        logger.debug('MOVING TOWARDS PRE-PLACE')
        pre_place_q = motion.find_valid_ik_solution(pre_place_pose)
        if not pre_place_q:
            logger.error("Impossible to calculate pre-place configuration.")
            return False

        path_to_pre_place = motion.plan_path_rrt(pre_place_q)
        if not path_to_pre_place:
            logger.error("Impossible to calculate path towards pre-place pose.")
            return False
        actuator.execute_path(path_to_pre_place)

        # 4. Final approach movement (linear, assumed safe)
        logger.debug("LOWERING VERTICALLY TO PLACE OBJECT")
        path_to_place = motion.plan_linear_path(place_pose)
        if not path_to_place:
            logger.error("Impossible to calculate path towards place pose.")
            return False
        actuator.execute_path(path_to_place)

        # 5. Object release
        gripper.open()
        gripper.detach_object(object_name)
        # time.sleep(0.01)

        # 6. Return
        logger.debug('LIFTING')
        lift_q = motion.find_valid_ik_solution(lift_pose)
        if not lift_q:
            logger.error("Impossible to calculate lift configuration.")
            return False

        path_to_lift = motion.plan_linear_path(lift_pose)
        if not path_to_lift:
            logger.error("Impossible to calculate path towards lift pose.")
            return False
        actuator.execute_path(path_to_lift)

        # 7. Return to rest position
        logger.debug("RETURNING TO INITIAL POSITION...")
        path_to_home = motion.plan_path_rrt(home_q)
        if not path_to_home:
            logger.error("Impossible to calculate path towards Home pose.")
            return False
        actuator.execute_path(path_to_home)

        return True

    def perform_nudge(self, object_name):
        client, sim, simIK, simOMPL = self.client, self.sim, self.simIK, self.simOMPL
        handles = self.handles
        planner = self.planner
        actuator = self.actuator

        # 1. Initial rest position
        home_q = [sim.getJointPosition(h) for h in handles['arm_joints']]

        # 2. Definition of target Cartesian poses relative to robot base
        logger.debug('CALCULATING TARGET POSES')
        nudge_pose = self.perception.get_nudge_pose(object_name)
        if not nudge_pose:
            logger.error("Impossible to calculate nudge-pose.")
            return False

        # PRE-GRASP: 15 cm above bounding box
        pre_nudge_pose = list(nudge_pose)
        pre_nudge_pose[3] += 0.15

        # # LIFT: Lift 20 cm relative to grasp
        # lift_pose = list(nudge_pose)
        # lift_pose[11] += 0.20

        # 3. Movement towards pre-nudge pose
        logger.debug('MOVING TOWARDS PRE-NUDGE')
        pre_nudge_q = planner.find_valid_ik_solution(pre_nudge_pose)
        if not pre_nudge_q:
            logger.error("Impossible to calculate pre-nudge configuration.")
            return False

        path_to_pre_nudge = planner.plan_path_rrt(pre_nudge_q)
        if not path_to_pre_nudge:
            logger.error("Impossible to calculate path towards pre-grasp pose.")
            return False
        actuator.execute_path(path_to_pre_nudge)

        path_to_nudge = planner.plan_linear_path(nudge_pose)
        if not path_to_nudge:
            logger.error("Impossible to calculate path towards grasp pose.")
            return False
        actuator.execute_path(path_to_nudge)

        # 7. Holding position for 3 seconds
        logger.debug("Holding object in position...")
        # for _ in range(60):  # 60 steps * 50ms/step = 3 seconds
        #     client.step()
        # time.sleep(3)

        # 8. Return to rest position
        path_to_home = planner.plan_path_rrt(home_q)
        if not path_to_home:
            logger.error("Impossible to calculate path towards Home pose.")
            return False
        actuator.execute_path(path_to_home)

        return True

    def perform_done(self):
        print("🤖 FAKE_EXECUTE: done (task finished)")
        return True

    def pick_success_detector(self, object_name):
        return self.perception.pick_success_detector(object_name)

    def place_success_detector(self, object_name, target_name):
        return self.perception.place_success_detector(object_name, target_name)

    def nudge_success_detector(self, object_name):
        print("🤖 FAKE_EXECUTE: nudge (task finished)")
        return True, "True"