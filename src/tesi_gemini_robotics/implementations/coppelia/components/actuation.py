import numpy as np
import time
import logging

logger = logging.getLogger(__name__)

class CoppeliaActuator:
    def __init__(self, client, sim, handles):
        self.client = client
        self.sim = sim
        self.handles = handles

    def execute_path(self, path, tolerance=0.01, timeout=20.0):
        """Executes a path defined as a list of joint configurations."""
        logger.debug(f"Executing path with {len(path)} waypoints...")
        sim = self.sim
        handles = self.handles

        for i, waypoint in enumerate(path):
            # Send target configuration to all joints
            for j, joint_handle in enumerate(handles['arm_joints']):
                sim.setJointTargetPosition(joint_handle, waypoint[j])

            # Synchronous control loop: advance simulation until target is reached
            start_time = time.time()
            while time.time() - start_time < timeout:
                # client.step()  # Advance one simulation step
                current_q = [sim.getJointPosition(h) for h in handles['arm_joints']]
                # Calculate distance (norm) between current config and target
                dist = np.linalg.norm(np.array(current_q) - np.array(waypoint))
                # print(f'dist={dist}') #DEBUG
                if dist < tolerance:
                    #print(f"Waypoint {i+1}/{len(path)} reached.")
                    break
                # Pause to avoid CPU overload
                time.sleep(0.05)
            else:
                logger.warning(f"  - ⚠️ TIMEOUT reached for waypoint {i + 1}.")
                return False

        logger.debug("✅ Path completed.")
        return True