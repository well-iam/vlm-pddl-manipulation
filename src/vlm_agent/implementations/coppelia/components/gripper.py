import logging
import time
logger = logging.getLogger(__name__)

# minOpen = 25mm
# maxOpen = 105mm (range = 80mm)
# travelSpeed = 50cm/s
# forceRange = [30,70] N

class CoppeliaGripper:
    def __init__(self, sim, handles):
        self.sim = sim
        self.handles = handles
        self.proxSensor_handle = self.sim.getObject('/Franka/FrankaGripper/attachProxSensor')
        logger.info(f"caught {self.sim.getObjectName(self.proxSensor_handle)}")
        time.sleep(3)

    def open(self):
        """Opens the gripper."""
        logger.debug("Opening gripper...")
        return self._set_gripper_velocity(open_gripper=True)

    def close(self):
        """Closes the gripper."""
        logger.debug("Closing gripper...")
        return self._set_gripper_velocity(open_gripper=False)

    def _set_gripper_velocity(self, open_gripper):
        """Internal method to actuate motors."""
        try:
            target_velocity = 0.02 if open_gripper else -0.02
            force = 20 if open_gripper else -20

            # Choose the second joint (first is "fictitious")
            openCloseJoint_handle = self.handles['gripper_joints'][1]
            joint_name = self.sim.getObjectAlias(openCloseJoint_handle) or self.sim.getObjectName(openCloseJoint_handle)
            logger.debug(f"Found gripper joint: {joint_name}")

            logger.debug(f"[BEFORE] Velocity set: {self.sim.getJointTargetVelocity(openCloseJoint_handle)}")
            logger.debug(f"[BEFORE] Max force set: {self.sim.getJointTargetForce(openCloseJoint_handle)}")
            logger.debug(f"[BEFORE] Measured force: {self.sim.getJointForce(openCloseJoint_handle)}")
            self.sim.setJointTargetVelocity(openCloseJoint_handle, target_velocity)
            self.sim.setJointTargetForce(openCloseJoint_handle, force)
            logger.debug(f"Velocity set: {self.sim.getJointTargetVelocity(openCloseJoint_handle)}")
            logger.debug(f"Max force set: {self.sim.getJointTargetForce(openCloseJoint_handle)}")
            logger.debug(f"Measured force: {self.sim.getJointForce(openCloseJoint_handle)}")
            return True

        except Exception as e:
            logger.exception(f"Gripper actuation error: ")
            return False

    def detect(self):
        res, distance, _, object_handle, _ = self.sim.handleProximitySensor(self.proxSensor_handle)
        time.sleep(2)
        self.sim.resetProximitySensor(self.proxSensor_handle)
        # Consider also checking distance<0.03
        if res:
            obj_name = self.sim.getObjectName(object_handle)
            logger.debug(f'Detected object "{obj_name}", at distance {distance}')
        else:
            logger.debug(f"No objects detected near gripper.")

        return res

    def attach_object(self, object_name):
        """
        SIMULATION TRICK: Makes the object a child of the gripper and updates collections
        to avoid self-collisions during transport.
        """
        try:
            logger.debug(f"Attempting logical attach (Attach) of '{object_name}'...")
            obj_handle = self.sim.getObject(f'/{object_name}')
            tip_handle = self.handles['tip']

            # 1. Proximity Check (Safety)
            # (Optional: verify object is truly near gripper before attaching)

            # 2. Set Parent
            self.sim.setObjectParent(obj_handle, tip_handle, True)  # True = KeepInPlace

            # 3. Update Collections (Crucial for OMPL)
            # Remove from Environment
            self.sim.addItemToCollection(self.handles['environment_collection'], self.sim.handle_single, obj_handle, 1)
            # Add to Robot
            self.sim.addItemToCollection(self.handles['robot_collection'], self.sim.handle_single, obj_handle, 0)
            # Add to Fake Robot
            self.sim.addItemToCollection(self.handles['fake_robot_collection'], self.sim.handle_single, obj_handle, 0)

            # Disable dynamics to avoid jittering while in hand
            self.sim.setBoolProperty(obj_handle, 'dynamic', False, 0)

            logger.info(f"Object '{object_name}' attached to gripper.")
            return True
        except Exception as e:
            logger.exception(f"Error during attach_object: ")
            return False

    def detach_object(self, object_name):
        """
        SIMULATION TRICK: Releases the object and restores physics/collections.
        """
        try:
            logger.debug(f"Attempting logical release (Detach) of '{object_name}'...")
            obj_handle = self.sim.getObject(f'/{object_name}')

            # 1. Remove Parent (becomes orphan/child of world)
            self.sim.setObjectParent(obj_handle, -1, True)

            # 2. Restore Collections
            # Remove from Robot
            self.sim.addItemToCollection(self.handles['robot_collection'], self.sim.handle_single, obj_handle, 1)
            # Remove from Fake Robot
            self.sim.addItemToCollection(self.handles['fake_robot_collection'], self.sim.handle_single, obj_handle, 1)
            # Add to Environment
            self.sim.addItemToCollection(self.handles['environment_collection'], self.sim.handle_single, obj_handle, 0)

            # 3. Reactivate Dynamics (so it falls/rests physically)
            self.sim.setBoolProperty(obj_handle, 'dynamic', True, 0)

            logger.info(f"Object '{object_name}' released.")
            return True
        except Exception as e:
            logger.error(f"Error during detach_object: {e}")
            return False