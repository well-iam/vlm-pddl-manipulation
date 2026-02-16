import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory

class FrankaActuation:
    def __init__(self, node):
        self.node = node
        self.logger = self.node.get_logger().get_child("actuation")
        self.logger.debug("--- Initialization of FrankaActuation component ---")

        # Action Client for Execute
        self._execute_client = ActionClient(
            self.node,
            ExecuteTrajectory,
            'execute_trajectory'
        )

    def execute_trajectory(self, trajectory):
        """
        Executes a previously calculated trajectory

        :param self: Description
        :param trajectory: Description
        """
        if not trajectory:
            return False

        if not self.operator_approval() == 'y':
            print("Execution cancelled by user")
            return False

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.logger.debug("Requesting trajectory execution...")
        send_goal_future = self._execute_client.send_goal_async(goal_msg)

        # Synchronous wait (since it is a fast calculation)
        rclpy.spin_until_future_complete(self.node, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.logger.error("Execution rejected by Server.")
            return False

        self.logger.debug("Execution accepted. Executing trajectory...")
        result_future = goal_handle.get_result_async()
        # Synchronous wait (since it is a fast calculation)
        rclpy.spin_until_future_complete(self.node, result_future)

        result = result_future.result().result
        status = result_future.result().status

        # moveit_msgs/MoveItErrorCodes
        # 1 = SUCCESS
        if result.error_code.val == 1:
            self.logger.debug(f"Planned execution successful!")
        else:
            self.logger.error(f"Execution failed. Error code: {result.error_code.val}")

        return result.error_code.val

    def operator_approval(self):
        return input(">>> Do you want to perform this move? (y/n): ")