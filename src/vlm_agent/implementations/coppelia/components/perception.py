import logging
import time
import math
from PIL import Image
import numpy as np

logger = logging.getLogger(__name__)

ROBOT_FINGER_PAD_HEIGHT = 0.015
MAX_FRANKA_REACH = 0.855

class CoppeliaPerception:
    def __init__(self, sim, handles):
        self.sim = sim
        self.handles = handles
        self.vision_sensor_handle = sim.getObject('/visionSensor')
        # Counter of taken images
        self.no_images = 0
        # INTERNAL STATE
        # Dictionary {object_name: handle} of EVERYTHING existing in the scene
        self.ground_truth_objects = self._scan_groud_truth_objects(['Floor', 'Franka', 'FakeFranka', 'Blue_Wall'])
        # List [object_names] of what was DETECTED in the last scan
        self.identified_objects_names = []

    def _scan_groud_truth_objects(self, ignore_list=None):
        """

        Queries the CoppeliaSim scene to identify all top-level objects
        (direct children of the scene).

        This method uses the correct and most efficient approach, leveraging
        the 'options' parameter of sim.getObjectsInTree function to delegate
        hierarchical filtering to the simulation engine.

        :return: A list of strings containing names of objects satisfying
                 all criteria.
        """
        logger.debug("Retrieving interactive top-level objects (Optimized Method)...")
        sim = self.sim
        # --- CORRECT LOGIC BASED ON DOCUMENTATION RESEARCH ---

        # 1. Get handles of ALL top-level objects in the scene.
        #    - treeBaseHandle = sim.handle_scene: Search starts from scene root.
        #    - objectType = sim.handle_all: Search is generalized to any
        #      object type for maximum robustness (not just 'shape').
        #    - options = 2: THIS IS THE KEY. Value 2 (bit 1 set)
        #      instructs the function to return ONLY direct children of root,
        #      excluding all sub-objects in deeper hierarchies.
        all_top_level_objects = sim.getObjectsInTree(sim.handle_scene, sim.sceneobject_shape, 2)  # sim.handle_all
        logger.debug(f"Found {len(all_top_level_objects)} total top-level objects in the scene.")

        # ----------------------------------------------------------------

        ground_truth_objects = {}
        for handle in all_top_level_objects:
            try:
                # Retrieve friendlier name: alias if exists, otherwise object name.
                # This is a best practice for object identification.[4]
                name = sim.getObjectAlias(handle)
                if name in ignore_list:
                    continue
                ground_truth_objects[name] = handle

            except Exception:
                logger.exception(f"\n  --> ⚠️ ERROR or irrelevant object {handle}: ")

        logger.info(
            f"Ground Truth updated: {len(ground_truth_objects)} known objects ({list(ground_truth_objects.keys())})")
        return ground_truth_objects

    def perceive_scene(self, gemini_client=None, use_vision=True):
        """
        Executes the complete perception cycle:
        1. Takes photo.
        2. (Optional) Asks Gemini what it sees.
        3. (Fallback) Uses Ground Truth if vision is disabled or fails.

        Populates self.identified_objects_names.
        """
        # 1. Acquire image (always useful for log or Gemini)
        image = self.get_camera_image()

        if use_vision and gemini_client:
            logger.info("Starting visual perception with Gemini...")
            # Pass known names list to Gemini to aid recognition (Closed-Set Recognition)
            known_names = list(self.ground_truth_objects.keys())

            # Call Gemini (assuming GeminiClient has this method)
            detected_names = gemini_client.get_object_list_from_image(image, known_names)

            if detected_names:
                # Filter: Accept only names that actually exist in Ground Truth (Anti-Hallucination)
                valid_detections = [name for name in detected_names if name in self.ground_truth_objects]
                self.identified_objects_names = valid_detections
                logger.info(f"Gemini saw: {self.identified_objects_names}")
            else:
                logger.warning("Gemini detected no objects. Using Ground Truth as fallback.")
                self.identified_objects_names = list(self.ground_truth_objects.keys())

        else:
            # "Blind" or "Debug" mode: Robot "sees" everything that exists (Oracle)
            logger.debug("Visual perception skipped. Using Ground Truth (Oracle Mode).")
            self.identified_objects_names = list(self.ground_truth_objects.keys())

        return self.identified_objects_names

    def _scan_scene_and_robot_state(self, objects_handles):
        """
        Builds a complete world state dictionary, including
        robot state (what it is holding) and environment state.
        """
        robot_base = self.handles['base']
        gripper_tip = self.handles['tip']

        # Final data structure
        world_state = {
            "robot_state": {"is_holding": "libero"},
            "environment_objects": {
                "manipulable_objects": [],
                "static_locations": []
            }
        }

        # Local cache to avoid re-calculations
        # Format: {handle: {'name': str, 'pos': [x,y,z], 'is_dynamic': bool}}
        objects_cache = {}

        # 1. FIRST PASS: Data collection and "is_holding" check
        # Identify immediately if holding something to exclude it from "resting on" logic
        held_object_handle = None

        for object_handle in objects_handles:
            # Get alias
            name = self.sim.getObjectAlias(object_handle)

            # Get physical properties
            is_dynamic = self.sim.getBoolProperty(object_handle, 'dynamic')
            is_respondable = self.sim.getBoolProperty(object_handle, 'respondable')

            # Get absolute position for fast calculations
            pos = self.sim.getObjectPosition(object_handle, -1)

            # Check if held by robot (hierarchy)
            parent = self.sim.getObjectParent(object_handle)
            if parent == gripper_tip:
                world_state["robot_state"]["is_holding"] = name
                # Do not add object to cache for environmental spatial calculations
                continue

            objects_cache[object_handle] = {
                'name': name,
                'pos': pos,
                'is_manipulable': is_dynamic and is_respondable  # or your combined logic
            }

        # 2. SECOND PASS: Build Scene Graph and Relations
        for object_handle, data in objects_cache.items():
            # Calculate distance from robot (using position relative to base for precision or absolute for estimate)
            dist_from_robot = self.dist(object_handle, robot_base)
            is_reachable = dist_from_robot <= MAX_FRANKA_REACH

            obj_info = {
                "name": data['name'],
                # "3D_position": [round(p, 3) for p in position],
                "distance_from_robot_m": round(dist_from_robot, 3) if dist_from_robot is not None else "N/A",
                "reachable": is_reachable
            }

            if data['is_manipulable']:
                # Calculate "resting_on" only for manipulable objects
                placed_on = self._placed_on_GEMINI(object_handle, objects_cache)

                obj_info["placed_on"] = placed_on
                world_state["environment_objects"]["manipulable_objects"].append(obj_info)
            else:
                # Static location
                world_state["environment_objects"]["static_locations"].append(obj_info)

        return world_state

    def get_world_state_data(self):
        logger.debug("Building World State...")
        identified_objects_handles = [self.ground_truth_objects.get(name) for name in self.identified_objects_names]
        return self._scan_scene_and_robot_state(identified_objects_handles)

    def get_camera_image(self):
        try:
            img, res = self.sim.getVisionSensorImg(self.vision_sensor_handle)
            img_np = np.frombuffer(img, dtype=np.uint8).reshape((res[1], res[0], 3))
            img_np = np.flipud(img_np)
            pil_image = Image.fromarray(img_np, 'RGB')
            pil_image.save(f"scatto_simulatore_{self.no_images}.png")
            self.no_images += 1
        except Exception as e:
            logger.exception(f"❌ Error acquiring image: {e}")
            return None
        return pil_image

    def get_grasp_pose(self, object_name):
        """
        Calculates the grasp pose using the Bounding Box (BB) for greater precision.
        """
        obj_handle = self.sim.getObject(f'/{object_name}')
        if not obj_handle: return None

        try:
            # 1. Get object base pose (its pivot) relative to robot base
            # 4x4 Matrix (flattened to 12 elements)
            object_matrix = self.sim.getObjectMatrix(obj_handle, self.handles['base'])

            # 2. Get Bounding Box (BB)
            # size: [x_size, y_size, z_size]
            # rel_pose: [x, y, z, qx, qy, qz, qw] BB center position relative to object pivot
            size, rel_pose = self.sim.getShapeBB(obj_handle)

            object_height = size[2]  # Z dimension is height

            # 3. Calculate TRUE GEOMETRIC CENTER
            # Often object pivot is at base, but we want the center.
            # getShapeBB tells us where center is relative to pivot.

            # If object is simple, we can approximate by summing relative Z offset
            # to object matrix Z position.
            # Note: For absolute precision matrix multiplication is needed,
            # but for non-rotated cubes and cylinders, summing local offset to global Z is sufficient.

            center_z_offset = rel_pose[2]

            # --- POSE DEFINITION ---

            # GRASP: Do we want gripper at object geometric center?
            # Or slightly above?
            grasp_pose = list(object_matrix)

            # Height correction:
            # Start from pivot Z (grasp_pose[11])
            # Add offset to reach geometric center (center_z_offset)
            # If you want to grab object "from above", might want to add something more (e.g. size[2]/2)
            # For now, aim for exact geometric center:
            grasp_pose[11] += center_z_offset

            # Grab object from "above"
            grasp_pose[11] += object_height/2.0 - ROBOT_FINGER_PAD_HEIGHT

            return grasp_pose

        except Exception as e:
            logger.error(f"Error calculating pose for '{object_name}': {e}")
            return None

    def get_place_pose(self, object_name, target_name):
        """
        Calculates the exact place pose so that the object
        rests on top of the target (stacking).

        The returned pose corresponds to the position the Gripper Tip must assume
        (assuming it grasped the object at the center).
        """
        obj_handle = self.sim.getObject(f'/{object_name}')
        target_handle = self.sim.getObject(f'/{target_name}')

        if not obj_handle or not target_handle:
            return None

        try:
            # 1. Get TARGET pose (e.g., Pad) relative to robot base
            # This matrix defines X, Y and base orientation
            target_matrix = self.sim.getObjectMatrix(target_handle, self.handles['base'])

            # 2. Get precise dimensions (Bounding Box)
            # size: [x, y, z], pose: [x, y, z, qx, qy, qz, qw] (relative to pivot)
            size_obj, pose_obj = self.sim.getShapeBB(obj_handle)
            size_tgt, pose_tgt = self.sim.getShapeBB(target_handle)

            obj_height = size_obj[2]
            tgt_height = size_tgt[2]

            # 3. Stacking Z Calculation
            # We want: Object_Bottom touching Target_Top
            logger.debug(f"target_matrix_z: {target_matrix[11]}")
            logger.debug(f"pose_tgt: {pose_tgt[2]}")
            logger.debug(f"target_height: {tgt_height}")
            # A. Find Z of Target top surface
            # Z_pivot_target + Offset_Center_BB + Half_Height
            target_surface_z = target_matrix[11] + pose_tgt[2] + (tgt_height / 2.0)
            logger.debug(f"target_surface_z: {target_surface_z}")

            # B. Find where Gripper/Object center must be
            # Target_Surface + Half_Object_Height
            # (Assuming grasp happened at geometric center of object)
            final_z = target_surface_z + (obj_height / 2.0)

            # 4. Construct Place matrix
            place_pose = list(target_matrix)
            place_pose[11] = final_z + obj_height/2.0 - ROBOT_FINGER_PAD_HEIGHT # (TODO) Sync with grasp point

            time.sleep(4.0)

            # Note: This will position object at X,Y center of target,
            # with same orientation as target.

            # 5. Return object height too, useful for calculating
            # safety distances (pre-place, lift) in caller.
            return place_pose, obj_height

        except Exception as e:
            logger.error(f"Error calculating place pose for '{object_name}' on '{target_name}': {e}")
            return None, 0.0

    def get_nudge_pose(self, object_name):
        """
        Calculates the grasp pose using the Bounding Box (BB) for greater precision.
        """
        obj_handle = self.sim.getObject(f'/{object_name}')
        if not obj_handle: return None

        try:
            # 1. Get object base pose (its pivot) relative to robot base
            # 4x4 Matrix (flattened to 12 elements)
            object_matrix = self.sim.getObjectMatrix(obj_handle, self.handles['base'])

            # 2. Get Bounding Box (BB)
            # size: [x_size, y_size, z_size]
            # rel_pose: [x, y, z, qx, qy, qz, qw] BB center position relative to object pivot
            size, rel_pose = self.sim.getShapeBB(obj_handle)

            object_height = size[2]  # Z dimension is height

            # 3. Calculate TRUE GEOMETRIC CENTER
            # Often object pivot is at base, but we want the center.
            # getShapeBB tells us where center is relative to pivot.

            # If object is simple, we can approximate by summing relative Z offset
            # to object matrix Z position.
            # Note: For absolute precision matrix multiplication is needed,
            # but for non-rotated cubes and cylinders, summing local offset to global Z is sufficient.

            center_z_offset = rel_pose[2]

            # --- POSE DEFINITION ---

            # GRASP: Do we want gripper at object geometric center?
            # Or slightly above?
            nudge_pose = list(object_matrix)

            # Height correction:
            # Start from pivot Z (grasp_pose[11])
            # Add offset to reach geometric center (center_z_offset)
            # If you want to grab object "from above", might want to add something more (e.g. size[2]/2)
            # For now, aim for exact geometric center:
            #nudge_pose[3] += 0.1
            return nudge_pose

        except Exception as e:
            logger.error(f"Error calculating pose for '{object_name}': {e}")
            return None

    def dist_xy(self, target_handle, ref_handle):
        target_pos = self.sim.getObjectPosition(target_handle)
        ref_pos = self.sim.getObjectPosition(ref_handle)
        xy_dist = math.sqrt(
            (target_pos[0] - ref_pos[0]) ** 2 +
            (target_pos[1] - ref_pos[1]) ** 2
        )
        return xy_dist

    def dist(self, target_handle, ref_handle):
        target_pos = self.sim.getObjectPosition(target_handle)
        ref_pos = self.sim.getObjectPosition(ref_handle)
        distance = math.sqrt(
            (target_pos[0] - ref_pos[0]) ** 2 +
            (target_pos[1] - ref_pos[1]) ** 2 +
            (target_pos[2] - ref_pos[2]) ** 2
        )
        return distance

    def is_on(self, object_handle, target_handle):
        sim = self.sim
        _, _, object_dimensions = sim.getShapeGeomInfo(object_handle)
        _, _, target_dimensions = sim.getShapeGeomInfo(target_handle)
        object_position = sim.getObjectPosition(object_handle, -1)
        object_z = object_position[2]
        target_position = sim.getObjectPosition(target_handle, -1)
        target_z = target_position[2]

        # distance = dist(sim, object_handle, target_handle)
        if self.dist_xy(object_handle, target_handle) < target_dimensions[0] / 2 and object_z > target_z:
            return True
        return False

    def _placed_on_GEMINI(self, object_handle, objects_cache):
        """
        Determines what an object is resting on by looking for the closest
        vertical candidate among those below it.
        """
        data = objects_cache[object_handle]

        # Default: if nothing found below, assume it's on table
        best_support_name = "table"

        # Initialize min distance found with high value (e.g. max threshold)
        # We are looking for object with smallest Z distance (closest underneath)
        min_z_distance = 0.20  # Max vertical threshold (10cm)

        for potential_support_handle, support_data in objects_cache.items():
            if object_handle == potential_support_handle:
                continue

            obj_z = data['pos'][2]
            supp_z = support_data['pos'][2]

            # Calculate vertical gap
            z_gap = obj_z - supp_z

            # 1. Z Filter: Object must be ABOVE support (z_gap > 0)
            # and must be WITHIN current threshold (or initial max)
            if 0 < z_gap < min_z_distance:

                # 2. XY Filter: Are they overlapping?
                dist_xy = ((data['pos'][0] - support_data['pos'][0]) ** 2 +
                           (data['pos'][1] - support_data['pos'][1]) ** 2) ** 0.5

                # Horizontal threshold (e.g., 10-15cm depending on object size)
                if dist_xy < 0.15:
                    # FOUND A BETTER CANDIDATE
                    # Do not break! Save this as new "best candidate"
                    # and tighten z_gap threshold for next iterations.
                    best_support_name = support_data['name']
                    min_z_distance = z_gap

        return best_support_name

    def pick_success_detector(self, object_name):
        object_handle = self.sim.getObject(f'/{object_name}')

        if self.dist(object_handle, self.handles['tip']) > 0.05:
            return False, "Object not picked"
        return True, "True"

    def place_success_detector(self, object_name, target_name):
        object_handle = self.sim.getObject(f'/{object_name}')
        target_handle = self.sim.getObject(f'/{target_name}')

        if self.is_on(object_handle, target_handle):
            return True, "True"
        return False, "Object not placed on target"