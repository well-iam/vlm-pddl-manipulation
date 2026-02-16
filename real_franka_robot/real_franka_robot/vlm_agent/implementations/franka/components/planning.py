import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import fundamental ROS messages
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    RobotState,
    RobotTrajectory,
    AttachedCollisionObject
)
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive


class FrankaPlanning:
    def __init__(self, node: Node, arm_group_name, ee_frame, joint_names):
        self.node = node
        self.logger = self.node.get_logger().get_child('planning')
        self.logger.debug("--- Initialization of FrankaPlanning component ---")

        self.arm_group_name = arm_group_name
        self.ee_frame = ee_frame
        self.joint_names = joint_names

        # Action Client for MoveGroup (Planning)
        self._move_action_client = ActionClient(
            self.node,
            MoveGroup,
            'move_action',
            #
        )

        # Specific publisher for attach/detach
        self._attached_object_pub = self.node.create_publisher(
            AttachedCollisionObject,
            '/attached_collision_object',
            10
        )

        # Service Client for Cartesian Planning
        self._cartesian_path_client = self.node.create_client(
            GetCartesianPath,
            'compute_cartesian_path'
        )

        # Connection to Server
        self._wait_for_moveit()

    def _wait_for_moveit(self):
        """Waits for MoveGroup action server to be available"""
        self.logger.debug("Waiting for 'move_action' server...")
        if not self._move_action_client.wait_for_server(timeout_sec=20.0):
            self.logger.error("Timeout: MoveIt Action Server not found. Verify that 'franka_moveit_config' is running.")
            raise RuntimeError("MoveIt not available")
        self.logger.debug("Server MoveIt connected.")

    def plan_trajectory_to_config(self, goal_config, velocity_scale=0.1, acceleration_scale=0.1) -> RobotTrajectory:
        """
        Plans a trajectory to a joint configuration.

        :param self: Description
        :param target_pose_stamped: Description
        :type target_pose_stamped: PoseStamped
        :param velocity_scale: Description
        :param acceleration_scale: Description
        """

        goal_msg = MoveGroup.Goal()

        # 1. Configuration planning parameters
        # (string) planner_id: the name of the planning algorithm to use. If no name is specified,
        # the default planner of the planning pipeline will be used
        goal_msg.request.group_name = self.arm_group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = velocity_scale
        goal_msg.request.max_acceleration_scaling_factor = acceleration_scale

        # Set start state to current state
        goal_msg.request.start_state = RobotState()
        goal_msg.request.start_state.is_diff = True

        # 2. Constraint Definition
        # MoveIt 2 requires explicit constraints for Position and Orientation
        constraints = Constraints()
        constraints.name = "config_goal"

        # Iterate over joints saved at beginning
        for i, position in enumerate(goal_config):
            # IMPORTANT: Filter joints. MoveIt fails if you pass gripper joints
            # when planning for "arm" group.
            jc = JointConstraint()
            jc.joint_name = self.joint_names[i]
            jc.position = position

            # Tolerances (Fundamental, otherwise it never finds exact goal)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0

            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)

        # 3. Execution Options
        goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True

        # 4. Send and Wait (Sync Pattern for LLM)
        self.logger.debug(f"Sending movement request to configuration [{goal_config}]...")

        send_goal_future = self._move_action_client.send_goal_async(goal_msg)

        # Synchronous wait (since it is a fast calculation)
        rclpy.spin_until_future_complete(self.node, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.logger.error("Planning rejected by Server.")
            return False

        self.logger.debug("Planning accepted.")
        result_future = goal_handle.get_result_async()
        # Synchronous wait (since it is a fast calculation)
        rclpy.spin_until_future_complete(self.node, result_future)

        result = result_future.result().result
        status = result_future.result().status

        # moveit_msgs/MoveItErrorCodes
        # 1 = SUCCESS
        if result.error_code.val == 1:
            planned_trajectory = result.planned_trajectory
            self.logger.debug(f"Plan calculated: {len(planned_trajectory.joint_trajectory.points)} waypoints")
            return planned_trajectory
        else:
            self.logger.error(f"Planning FAILED. Error code: {result.error_code.val}")
            return None

    def plan_trajectory(self, target_pose_stamped: PoseStamped, velocity_scale=0.1,
                        acceleration_scale=0.1) -> RobotTrajectory:
        """
        Plans a trajectory to a pose.

        :param self: Description
        :param target_pose_stamped: Description
        :type target_pose_stamped: PoseStamped
        :param velocity_scale: Description
        :param acceleration_scale: Description
        """

        goal_msg = MoveGroup.Goal()

        # 1. Configuration planning parameters
        # (string) planner_id: the name of the planning algorithm to use. If no name is specified,
        # the default planner of the planning pipeline will be used
        goal_msg.request.group_name = self.arm_group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = velocity_scale
        goal_msg.request.max_acceleration_scaling_factor = acceleration_scale

        # Set start state to current state
        goal_msg.request.start_state = RobotState()
        goal_msg.request.start_state.is_diff = True

        # 2. Constraint Definition
        # MoveIt 2 requires explicit constraints for Position and Orientation
        constraints = Constraints()
        constraints.name = "llm_generated_goal"

        # Position Constraint
        p_const = PositionConstraint()
        p_const.header = target_pose_stamped.header
        p_const.link_name = self.ee_frame
        p_const.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005]))  # 5mm tolerance
        p_const.constraint_region.primitive_poses.append(target_pose_stamped.pose)
        p_const.weight = 1.0

        # Orientation Constraint
        o_const = OrientationConstraint()
        o_const.header = target_pose_stamped.header
        o_const.link_name = self.ee_frame
        o_const.orientation = target_pose_stamped.pose.orientation
        # Angular Tolerances
        # TILT (Verticality): Severe to prevent gripper arriving tilted
        o_const.absolute_x_axis_tolerance = 0.01
        o_const.absolute_y_axis_tolerance = 0.01
        # YAW (Rotation around vertical): Slightly more permissive (auto-centering)
        # Helps solver find solution faster
        o_const.absolute_z_axis_tolerance = 0.02
        o_const.weight = 1.0

        constraints.position_constraints.append(p_const)
        constraints.orientation_constraints.append(o_const)
        goal_msg.request.goal_constraints.append(constraints)

        # 3. Execution Options
        goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True

        # 4. Send and Wait (Sync Pattern for LLM)
        target_pos = target_pose_stamped.pose.position
        self.logger.debug(f"Sending movement request to ({target_pos.x:.3f}, {target_pos.y:.3f}, {target_pos.z:.3f})")

        send_goal_future = self._move_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        """ # Comment
        while not send_goal_future.done():
            time.sleep(0.1)
            pass # TODO: use better wait conditions or async/await """

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.logger.error("Planning rejected by Server.")
            return False

        self.logger.debug("Planning accepted.")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        """ while not result_future.done():
            time.sleep(0.1) """

        result = result_future.result().result
        status = result_future.result().status

        # moveit_msgs/MoveItErrorCodes
        # 1 = SUCCESS
        if result.error_code.val == 1:
            planned_trajectory = result.planned_trajectory
            self.logger.debug(f"Plan calculated: {len(planned_trajectory.joint_trajectory.points)} waypoints")
            return planned_trajectory
        else:
            self.logger.error(f"Planning FAILED. Error code: {result.error_code.val}")
            return None

    # TODO: what is the difference between using get_current_pose and RobotState() with use_diff?
    # TODO: how to set velocity_scaling and accelerate_scaling here?
    def plan_linear_path(self, target_pose_stamped: PoseStamped, ignore_collisions=False,
                         threshold=1.0) -> RobotTrajectory:
        """
        Plans a linear Cartesian path.

        :param self: Description
        :param target_pose_stamped: Description
        :type target_pose_stamped: PoseStamped
        :return: Description
        :rtype: RobotTrajectory
        """

        self.logger.debug(f"Calculating LINEAR path to: {target_pose_stamped.pose.position}")

        # 1. Wait Service
        if not self._cartesian_path_client.wait_for_service(timeout_sec=2.0):
            self.logger.error("Service 'compute_cartesian_path' not available.")
            return None

        # 2. Request Construction
        req = GetCartesianPath.Request()
        req.header = target_pose_stamped.header
        req.start_state = RobotState()
        req.start_state.is_diff = True  # Start from current state

        req.group_name = self.arm_group_name
        req.link_name = self.ee_frame

        # List of waypoints (here we put only one, the final target)
        # MoveIt will interpolate straight line between "here" and this point.
        req.waypoints = [target_pose_stamped.pose]

        # Interpolation parameters
        req.max_step = 0.02  # Resolution: one point every 1 cm
        req.jump_threshold = 5.0  # 0.0 disables jump check (or use e.g. 5.0 for safety)
        req.avoid_collisions = not ignore_collisions  # Try to avoid collisions along the line

        # 3. Call service
        future = self._cartesian_path_client.call_async(req)

        # Synchronous wait (since it is a fast calculation)
        rclpy.spin_until_future_complete(self.node, future)

        response = future.result()
        # 4. Result Validation
        if response.error_code.val != 1:
            self.logger.error(f" Cartesian calculation error: {response.error_code.val}")
            return None

        # Fraction Check: MoveIt might return partial path (e.g. 50% of way)
        # if it meets joint limit. We want only complete paths
        if response.fraction < threshold:
            self.logger.warn(
                f"Linear path incomplete! Calculated only {response.fraction * 100:.1f}% on threshold {threshold * 100:.1f}%. Ignored.")
            return None

        self.logger.debug(f"Linear plan calculated ({len(response.solution.joint_trajectory.points)} points.)")

        # Returns object of type moveit_msgs/RobotTrajectory
        return response.solution