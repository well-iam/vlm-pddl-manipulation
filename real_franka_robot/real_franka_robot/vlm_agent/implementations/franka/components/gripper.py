import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from franka_msgs.action import Grasp, Move, Homing

class FrankaGripper:
    def __init__(self, node: Node):
        self.node = node
        self.logger = self.node.get_logger().get_child("gripper")
        self.logger.debug("--- Initialization of FrankaGripper component ---")

        self.current_width = 0.0

        # Subscribe to joint states to always know gripper opening
        self._joint_sub = self.node.create_subscription(
            JointState,
            '/franka_gripper/joint_states',
            self._joint_state_callback,
            10
        )

        # Client for Franka Gripper ('namespace /franka_gripper)
        self._homing_client = ActionClient(
            self.node, Homing, '/franka_gripper/homing'
        )

        self._move_client = ActionClient(
            self.node, Move, '/franka_gripper/move'
        )

        self._grasp_client = ActionClient(
            self.node, Grasp, '/franka_gripper/grasp'
        )

    def _joint_state_callback(self, msg: JointState):
        self.width = sum([abs(p) for p in msg.position])

    def get_current_width(self):
        return self.current_width

    def homing(self):
        """
        Calibrates the gripper. Necessary after power-on or reflex error.
        """
        self.logger.debug("Executing Gripper Homing...")

        if not self._homing_client.wait_for_server(timeout_sec=2.0):
            self.logger.error("Action Server /franka_gripper/homing not found!")
            return False

        goal = Homing.Goal()
        future = self._homing_client.send_goal_async(goal)

        # Synchronous wait (since it is a fast calculation)
        rclpy.spin_until_future_complete(self.node, future)

        # Wait for action result
        result_future = future.result().get_result_async()
        # Synchronous wait (since it is a fast calculation)
        rclpy.spin_until_future_complete(self.node, result_future)

        self.logger.debug("Homing completed.")
        return True

    def open(self, width=0.08, speed=0.1):
        """
        Opens gripper to specific width (default 8cm = max).
        """
        self.logger.debug(f"Opening gripper to {width}m...")

        if not self._move_client.wait_for_server(timeout_sec=1.0):
            self.logger.error("Move Server not found")
            return False

        goal = Move.Goal()
        goal.width = width  # [m]
        goal.speed = speed  # [m/s]

        future = self._move_client.send_goal_async(goal)
        # Return acceptance
        # Synchronous wait (since it is a fast calculation)
        rclpy.spin_until_future_complete(self.node, future)

        res = future.result().get_result_async()
        # Synchronous wait (since it is a fast calculation)
        rclpy.spin_until_future_complete(self.node, res)

        result = res.result().result
        if result.success:
            return True
        else:
            self.logger.warn(f"Gripper open failed (or already open): {result.error}")
            return False

    def grasp(self, width=0.04, force=1.0, speed=0.1, epsilon=0.0375):
        """
        Attempts to grasp an object.
        Returns True if something grasped, False if closed on nothing.
        """
        self.logger.debug(f"Attempting grasp: Width{width}m, Force={force}N")

        if not self._grasp_client.wait_for_server(timeout_sec=1.0):
            self.logger.error("Grasp Server not found")
            return False

        goal = Grasp.Goal()
        goal.width = width  # Expected object size
        goal.speed = speed
        goal.force = force

        # Tolerance: defines when a grasp is considered "Success"
        # Inner: how much smaller object can be
        # Outer: how much larger object can be
        goal.epsilon.inner = epsilon
        goal.epsilon.outer = epsilon

        future = self._grasp_client.send_goal_async(goal)
        # Received acceptance
        # Synchronous wait (since it is a fast calculation)
        rclpy.spin_until_future_complete(self.node, future)

        res_future = future.result().get_result_async()
        # Synchronous wait (since it is a fast calculation)
        rclpy.spin_until_future_complete(self.node, res_future)

        result = res_future.result().result
        if result.success:
            # self.logger.info("Grasp Successful!")
            return True
        else:
            self.logger.warn(f"Grasp failed: {result.error}")
            return False