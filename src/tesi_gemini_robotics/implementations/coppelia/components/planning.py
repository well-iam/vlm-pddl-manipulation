import logging
import time
import numpy as np

logger = logging.getLogger(__name__)

class CoppeliaPlanner:
    def __init__(self, sim, simIK, simOMPL, scene_handles):
        self.sim = sim
        self.simIK = simIK
        self.simOMPL = simOMPL
        self.scene_handles = scene_handles

        self.ik_handles = self._setup_ik_environment()
        if not self.ik_handles:
            raise RuntimeError("Unable to initialize movement system (IK Setup failed).")

    def _setup_ik_environment(self):
        """
        Creates the IK environment and TRANSLATES scene handles into IK handles.
        """
        logger.info("Setting up Inverse Kinematics (IK) environment...")
        simIK = self.simIK
        scene_handles = self.scene_handles

        try:
            # 1. Create IK environment and group
            ik_env = simIK.createEnvironment()
            ik_group = simIK.createGroup(ik_env)
            # simIK.setGroupCalculation(ik_env, ik_group, simIK.method_pseudo_inverse, 0, 6)

            # 2. Add robot kinematic chain to IK world
            #    This function is key: takes SCENE handles...
            ik_element, sim_to_ik_map, ik_to_sim_map = simIK.addElementFromScene(
                ik_env,
                ik_group,
                scene_handles['base'],
                scene_handles['tip'],
                scene_handles['target'],
                simIK.constraint_pose  # Constrain both position and orientation
            )
            # print(ik_element)
            # print(sim_to_ik_map)
            # print(ik_to_sim_map)
            # print(scene_handles['arm_joints'])
            # 3. TRANSLATION: Use map to find IK joint handles
            #    Take scene joint handles and find their "twins" in IK world.
            ik_joint_handles = [sim_to_ik_map[h] for h in scene_handles['arm_joints']]

            # ADDED
            ik_base_handle = sim_to_ik_map[scene_handles['base']]
            ik_tip_handle = sim_to_ik_map[scene_handles['tip']]
            ik_target_handle = sim_to_ik_map[scene_handles['target']]

            # check_scene_joint_handles = [ik_to_sim_map[h] for h in ik_joint_handles]
            # print(check_scene_joint_handles)
            # print(scene_handles)

            # 4. Save everything in a dictionary for future use
            ik_handles = {
                'env': ik_env,
                'group': ik_group,
                'element': ik_element,
                'joints_ik': ik_joint_handles,  # THIS is the correct list to use in findConfigs
                'base_ik': ik_base_handle,
                'tip_ik': ik_tip_handle,
                'target_ik': ik_target_handle
            }

            logger.info("✅ IK environment created and handles translated successfully.")
            return ik_handles

        except Exception as e:
            logger.exception(f"❌ ERROR during IK environment configuration: ")
            return None

    def _set_dummy_config(self, config_to_check):
        for i, h in enumerate(self.scene_handles['fake_arm_joints']):
            self.sim.setJointPosition(h, config_to_check[i])

    def _get_dummy_config(self):
        return [self.sim.getJointPosition(h) for h in self.scene_handles['fake_arm_joints']]

    def _enforce_limits(self, config, safety_margin=0.001):
        """
        Sanitizes a joint configuration ensuring it respects limits
        imposed in CoppeliaSim. Replaces simOMPL.enforceBounds.
        """
        sim = self.sim
        clamped_config = list(config)
        joints = self.scene_handles['arm_joints']

        for i, joint_handle in enumerate(joints):
            # Get joint interval [min, range]
            # Note: getJointInterval returns (bool, interval)
            is_cyclic, interval = sim.getJointInterval(joint_handle)

            # Check if joint is cyclic (e.g., continuous rotation)
            # Cyclic joints don't have hard limits, so we skip them
            if not is_cyclic:
                min_val = interval[0]
                max_val = min_val + interval[1]  # interval[1] is amplitude (range)

                # Apply safety margin
                safe_min = min_val + safety_margin
                safe_max = max_val - safety_margin

                # Clamping
                if clamped_config[i] < safe_min:
                    clamped_config[i] = safe_min
                elif clamped_config[i] > safe_max:
                    clamped_config[i] = safe_max

        return clamped_config

    def _is_config_valid_with_dummy(self, config_to_check):
        """
        Uses DUMMY robot to verify if a given configuration is valid.
        This function is safe because it operates on a non-dynamic model.
        """
        sim = self.sim
        scene_handles = self.scene_handles
        try:
            # 1. Apply configuration to DUMMY robot
            current_config = self._get_dummy_config()
            self._set_dummy_config(config_to_check)
            time.sleep(3)

            # 2. Check collisions for DUMMY
            collision_env, collision_handles = sim.checkCollision(scene_handles['fake_robot_collection'], scene_handles['environment_collection'])
            self._set_dummy_config(current_config)
            # obj1_name = sim.getObjectAlias(handles[0]) or sim.getObjectName(handles[0])
            # obj2_name = sim.getObjectAlias(handles[1]) or sim.getObjectName(handles[1])
            #auto_collision = sim.checkCollision(handles['fake_robot_collection'], handles['fake_robot_collection'])
            return collision_handles, not (collision_env) #or auto_collision)

        except Exception as e:
            logger.exception(f"  - Error during validation with dummy: ")
            return False

    # TODO
    def solve_ik(simIK, ik_handles, target_pose):
        """
        Solves inverse kinematics for a given Cartesian target pose.
        Returns a valid joint configuration or None.
        """
        logger.debug("Solving Inverse Kinematics (IK)...")
        current_pos = simIK.getObjectMatrix(ik_handles['env'], ik_handles['target_ik'], ik_handles['base_ik'])
        #print(f'CURRENT_POS: {current_pos}')

        simIK.setObjectMatrix(ik_handles['env'], ik_handles['target_ik'], target_pose, ik_handles['base_ik'])
        # changed_pos = simIK.getObjectMatrix(ik_handles['env'], ik_handles['target_ik'], ik_handles['base_ik'])
        #print(f'CHANGED_POS: {changed_pos}')

        # Perform IK search
        # Now perform search using IK WORLD handles
        # res, reason, _ = simIK.handleGroup(ik_handles['env'], ik_handles['group'])
        # if res!= simIK.result_success:
        #     print(f'Reason: {reason}')
        #     return None
        joint_configs = simIK.findConfigs(
            ik_handles['env'],
            ik_handles['group'],
            ik_handles['joints_ik'],  # <-- Pass list of translated IK handles!
            {}
        )
        logger.debug(f'joint_configs: {joint_configs}') #DEBUG

        if joint_configs:
            logger.debug(f"  - Found {len(joint_configs)} IK solutions. Using the first one.")
            # Function returns a list of solutions.
            # We take the first one, which is closest to current configuration.
            return joint_configs[0]
        else:
            logger.error("❌ No IK solution found for specified pose.")
            return None

    def find_valid_ik_solution(self, target_pose_matrix, max_attempts=10, search_time_per_attempt=1.0):
        """
        Searches for an IK solution and validates it immediately for collisions.
        Retries finding alternative solutions if the first one is invalid.
        """
        logger.debug("Searching for VALID IK solution...")
        simIK = self.simIK
        ik_handles = self.ik_handles

        # Set target pose in SCENE world (on real robot dummy target)
        simIK.setObjectMatrix(ik_handles['env'], ik_handles['target_ik'], target_pose_matrix, ik_handles['base_ik'])

        for attempt in range(max_attempts):
            logger.debug(f"  - Attempt {attempt + 1}/{max_attempts}...")

            # 1. Perform IK search
            #    Function can return multiple geometric solutions (e.g. elbow up/down)
            joint_configs = simIK.findConfigs(
                ik_handles['env'],
                ik_handles['group'],
                ik_handles['joints_ik'],
                {'maxTime': search_time_per_attempt}  # Parameter to limit search time
            )

            if not joint_configs:
                logger.debug(""""
                    - IK did not find geometric solutions in this attempt.
                    A reason for a non-successful operation can be: 
                     - there are some forbidden poses/configurations on the way
                     - some of the configuration points cannot be reached (e.g. out of reach, or due to joint limits).""")
                continue  # Skip to next attempt

            logger.debug(f"    - Found {len(joint_configs)} geometric solutions. Validating now...")
            # 2. Iterate over ALL solutions found and test the first valid one
            for i, solution_q in enumerate(joint_configs):
                logger.debug(f"      - Validating solution {i + 1}...")
                # 3. Use DUMMY robot for collision check
                collision_handles, config_valid = self._is_config_valid_with_dummy(solution_q)
                if config_valid:
                    logger.debug(f"    - ✅ Found a VALID and collision-free solution at attempt {attempt + 1}!")
                    return solution_q  # Return first valid solution found
                else:
                    logger.warning(f"      - ❌ Collision between {collision_handles}.")

        # If loop finishes without returning a solution, it means all attempts failed.
        logger.error(f"❌ FAILURE: No valid and collision-free IK solution found after {max_attempts} attempts.")
        return None

    def plan_linear_path(self, target_pose):
        """
        Solves inverse kinematics for a given Cartesian target pose.
        Returns a valid joint configuration or None.
        """
        logger.debug("Calculating Cartesian path with simIK.generatePath...")
        simIK = self.simIK
        ik_handles = self.ik_handles

        simIK.syncFromSim(ik_handles['env'], [ik_handles['group']])
        simIK.setObjectMatrix(ik_handles['env'], ik_handles['target_ik'], target_pose, ik_handles['base_ik'])
        # Desired number of points in resulting path
        number_of_path_points = 10

        try:
            # Call function passing correct IK handles and callback
            config_list_flat = simIK.generatePath(
                ik_handles['env'],  # Handle of IK environment
                ik_handles['group'],  # Handle of IK group
                ik_handles['joints_ik'],  # IK handles of joints (for output)
                ik_handles['tip_ik'],  # SCENE handle (???) of tip (required by function)
                number_of_path_points,  # How many points to generate
                #lambda state: is_state_valid(sim, scene_handles, state)  # Our callback
            )

            if config_list_flat:

                # Reorganize flat output into list of waypoints
                num_joints = len(ik_handles['joints_ik'])
                num_points = len(config_list_flat) // num_joints
                path_waypoints = [config_list_flat[i * num_joints: (i + 1) * num_joints] for i in range(num_points)]
                logger.debug(f"✅ Cartesian path found with {len(path_waypoints)} waypoints.")
                self._set_dummy_config(path_waypoints[-1])
                return path_waypoints

            else:
                logger.error("❌ simIK.generatePath failed to find a valid path.")
                return None

        except Exception as e:
            logger.exception(f"❌ Error during simIK.generatePath execution: ")

    def plan_path_rrt(self, goal_config):
        """Plans a collision-free path using OMPL with RRTConnect algorithm."""
        logger.debug("Planning path with RRTConnect...")
        sim = self.sim
        simOMPL = self.simOMPL
        handles = self.scene_handles

        start_config = [sim.getJointPosition(h) for h in handles['arm_joints']]
        # Definition of state space (7 arm joints)
        #state_space = simOMPL.createStateSpace('j', simOMPL.StateSpaceType.real, handles['arm_joints'], [-3.14, 3.14])

        # Create planning task
        task = simOMPL.createTask('task')

        # Algorithm setup
        simOMPL.setAlgorithm(task, simOMPL.Algorithm.BiTRRT)  # or RRTConnect, or RRTstar

        # STATE SPACE CREATION AND ASSIGNMENT
        # PROJECTION DEFINITION
        # Create flag list. Set first two joints to 1,
        # telling OMPL to use these for its "2D map".
        # Usually first 2 or 3 joints are chosen, as they determine widest arm movements.
        projection_setup = [1, 1, 0, 0, 0, 0, 0]  # Example: [1, 1, 0, 0, 0, 0, 0]
        # CREATING STATE SPACE WITH PROJECTION
        simOMPL.setStateSpaceForJoints(task, handles['arm_joints'], projection_setup)

        # This function is the heart of collision checking.
        def state_validity_callback(state):
            # Save current robot pose to restore it
            # original_q = [sim.getJointPosition(h) for h in handles['arm_joints']]

            # Temporarily set robot to 'state' configuration to test
            self._set_dummy_config(state)

            # Execute collision check using main CoppeliaSim engine
            is_colliding_RE, _ = sim.checkCollision(handles['fake_robot_collection'], handles['environment_collection'])
            if is_colliding_RE:
                print(f'COLLISION Robot-Environment: {is_colliding_RE}')

            is_colliding_RR, _ = sim.checkCollision(handles['fake_robot_collection'], handles['fake_robot_collection'])
            if is_colliding_RR:
                print(f'COLLISION Robot-Robot: {is_colliding_RR}')

            # Restore original robot pose
            # self._set_dummy_config(original_q)

            # Returns True if NO collision (valid state)
            return not (is_colliding_RE or is_colliding_RR)

        # Setting collision pairs: robot must not collide with environment
        # simOMPL.setCollisionPairs(task, [
        #     handles['robot_collection'], handles['environment_collection']])
        #     # handles['robot_collection'], handles['robot_collection']])    # Gives "invalid state", maybe use custom callback

        simOMPL.setStateValidationCallback(task, state_validity_callback)

        # Setting initial and final states
        # Sanitize initial state
        logger.debug(f"Initial state (raw): {np.round(start_config, 4)}")
        valid_start_config = self._enforce_limits(start_config)
        logger.debug(f"Initial state (enforced): {np.round(valid_start_config, 4)}")

        simOMPL.setStartState(task, valid_start_config)
        simOMPL.setGoalState(task, goal_config)

        # Setup task
        simOMPL.setup(task)

        # Execute planning
        solved, path_raw = simOMPL.compute(task, 5.0)  # 5 seconds max time
        # print(f'SOLVED: {solved}')
        # print(f'LENGTH PATH: {len(path)}')

        # Cleanup
        simOMPL.destroyTask(task)

        if solved:
            self._set_dummy_config(goal_config)
            # Output is flat list [p1_j1, p1_j2..., p2_j1, p2_j2...].
            # Must group into list of lists (waypoints).
            num_joints = len(handles['arm_joints'])
            num_points = len(path_raw) // num_joints
            path = [path_raw[i * num_joints: (i + 1) * num_joints] for i in range(num_points)]
            logger.debug(f"✅ RRT path found with {len(path)} waypoints.")
            return path
        else:
            logger.error("❌ RRT planning failed. No path found.")
            return None