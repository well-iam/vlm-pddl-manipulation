import logging
import json
import re
import cv2
import os
from datetime import datetime
import textwrap
import tempfile
import shutil
import time
from unified_planning.io import PDDLReader
from unified_planning.plans import SequentialPlan, ActionInstance
from unified_planning.shortcuts import PlanValidator

logger = logging.getLogger(__name__)

class TaskExecutor:

    def __init__(self, gemini_planner, robot, pddl_domain_path, max_steps=10):
        self.gemini_planner = gemini_planner
        self.robot = robot
        self.max_steps = max_steps  # Safety limit to avoid infinite loops in execute_autonomous_task
        self.pddl_domain_path = pddl_domain_path

        # Internal task state
        self.last_action_feedback = "No previous action."
        self.current_plan = []
        self.world_state = None

        # Annotation ID map with MoveIt names
        self.visual_id_to_moveit_map = {}

        # Folder for images
        self.log_dir = "logs/vision_history"
        os.makedirs(self.log_dir, exist_ok=True)

    def execute(self, goal, strategy="planning"):
        """
        Unified method to launch the task with the chosen strategy.
        """
        logger.info("Starting execution in PLANNING mode (Plan-and-Execute)...")
        return self.execute_plan(goal)

    def _update_perception(self):
        """
        Main wrapper (The Bridge).

        Returns:
            dict: A standardized dictionary containing 'rgb', 'depth', 'proprioception', etc.
        """
        observation = self._update_perception_franka()

        time.sleep(3.0)
        # (Optional) Here you can perform a common check or logging
        return observation

    def _update_perception_franka(self):
        """
        Internal method to update perception and save state.
        Returns the JSON string ready for the prompt.
        """

        """
        Real-World Sensory Fusion:
        1. GEOMETRY: From ROS service (precise positions, anonymous objects).
        2. SEMANTICS: From Gemini (looking at the image and correlating positions).
        3. ROBOT STATE: From FrankaGripper module (internal memory).
        """

        # ---------------------------------------------------------
        # 1. ROBOT STATE RETRIEVAL (ABSOLUTE PRIORITY)
        # ---------------------------------------------------------
        # The camera cannot see the gripper, so we blindly trust the memory
        # of the control module you mentioned.
        # I assume self.gripper_module has methods like get_status()
        gripper_memory = self.robot.gripper_state

        # gripper_memory should be a dict like:
        # {'is_holding': True, 'held_object_name': 'cubo_rosso', 'width': 0.04}

        robot_state_json = {
            "is_holding": gripper_memory['held_object_name'] if gripper_memory['is_holding'] else "libero"
        }

        # ---------------------------------------------------------
        # 2. GEOMETRIC OBJECT RETRIEVAL (ROS)
        # ---------------------------------------------------------
        # Call to existing service returning collision objects
        # Returns list of dicts: [{'id': 'collision_0', 'pos': [0.5, 0.2, 0.1], 'dims': [...]}, ...]
        """raw_scene_objects = self.robot.perception.get_scene_objects()"""
        # NOTE: Should not be useful
        # We filter out the object we are holding (if it exists), because
        # geometrically it moves with the robot and is not part of the "static environment" to plan
        """ env_objects_geometry = [
            obj for obj in raw_scene_objects 
            if obj['id'] != gripper_memory.get('held_object_name')
        ] """

        # ^---------------------- TODO: Remember to integrate info on gripper too? Or let it deduce via logic?
        cv_annotated_image, visual_id_to_moveit_map = self.robot.get_annotated_image_and_map()
        self.visual_id_to_moveit_map = visual_id_to_moveit_map
        return cv_annotated_image


    def _parse_pddl_gemini_response(self, response_text):
        """
        Extracts the PDDL block and JSON block from the LLM response.
        """

        # 1. PDDL Extraction (searches for text between ```pddl and ```)
        pddl_pattern = r"```pddl\s*(.*?)\s*```"
        pddl_match = re.search(pddl_pattern, response_text, re.DOTALL)

        problem_pddl_content = ""
        if pddl_match:
            problem_pddl_content = pddl_match.group(1).strip()
            print("✅ PDDL Problem extracted.")
        else:
            print("⚠️ WARNING: No PDDL block found in the answer.")

        # 2. JSON Extraction (searches for text between ```json and ```)
        json_pattern = r"```json\s*(.*?)\s*```"
        json_match = re.search(json_pattern, response_text, re.DOTALL)

        plan_json = {}
        if json_match:
            json_str = json_match.group(1).strip()
            try:
                plan_json = json.loads(json_str)
                print("✅ JSON extracted.")
            except json.JSONDecodeError as e:
                print(f"❌ Error parsing JSON: {e}")
        else:
            # Fallback: sometimes Gemini responds with naked JSON without backticks
            try:
                # Desperate attempt: look for first open brace and last close brace
                start = response_text.find('{')
                end = response_text.rfind('}') + 1
                if start != -1 and end != -1:
                    plan_json = json.loads(response_text[start:end])
                    print("⚠️ JSON extracted without markdown (fallback).")
            except:
                print("❌ Unable to find valid JSON.")

        return problem_pddl_content, plan_json

    def execute_plan(self, high_level_goal):
        """
        Executes an autonomous task:
        1. Plans a complete plan.
        2. Executes the plan step-by-step.
        3. If a step fails, stops and replans.
        """
        logger.info(f"======= EXECUTING AUTONOMOUS TASK: '{high_level_goal}' =======")
        gemini_client = self.gemini_planner

        # --- EXECUTION CYCLE STATE ---
        self.current_plan = []  # The list of steps to execute
        self.last_action_feedback = "No previous action. This is the first step."
        task_completed = False

        MAX_RETRIES = 3

        # Start chat. System instruction will be formatted and sent in the loop
        gemini_client.start_chat()  # Resets history

        while not task_completed:
            # --- 1. PLANNING OR REPLANNING PHASE ---
            if not self.current_plan:  # If we don't have a plan, create one
                logger.info("No active plan. Generating a new plan...")

                # World perception (gets annotated OpenCV image) and internally updates ID map
                cv_annotated_image = self._update_perception()
                if cv_annotated_image is None:
                    return False
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(f"{self.log_dir}/{timestamp}.jpg", cv_annotated_image)
                # Conversion: OpenCV -> Bytes
                # Compression can happen here
                success, encoded_img = cv2.imencode('.jpg', cv_annotated_image)
                if not success:
                    logger.error("Image conversion failed.")
                    continue
                image_bytes = encoded_img.tobytes()

                # Plan Request
                base_prompt = textwrap.dedent(f"""          
                *** Previous Action Feedback: ***
                {self.last_action_feedback}

                *** Goal: ***
                "{high_level_goal}"
                """).strip()

                # --- START RETRY LOOP ---
                current_prompt = base_prompt
                planning_succeded = False

                for retry in range(1, MAX_RETRIES + 1):
                    logger.info(f"--- Planning attempt {retry}/{MAX_RETRIES} ---")

                    # API call check (optional, to save tokens during debug)
                    send_prompt = input("Do you want to send the prompt to Gemini (CALLS ARE LIMITED!) (y/n): ")
                    if send_prompt != "y": break

                    vlm_answer = gemini_client.send_chat_message(current_prompt, image_bytes=image_bytes)

                    # 1. Syntactic Parsing (Broken JSON/PDDL)
                    try:
                        problem_code, response_json = self._parse_pddl_gemini_response(vlm_answer.text)
                        self.process_vlm_response(response_json)  # updates path to domain.pddl if requested by vlm
                    except Exception as e:
                        error_msg = f"Syntax error in JSON/PDDL: {str(e)}"
                        logger.warning(error_msg)
                        # Feedback for next attempt
                        current_prompt = f"{base_prompt}\n\n⚠️ CRITICAL ERROR: Your previous answer contained invalid syntax. Please correct this error:\n{error_msg}"
                        continue  # Retry

                    # 2. Logic Validation (PDDL Validator)
                    # Note: Modify validate_plan_with_pddl to return (bool, error_string)
                    is_valid, validation_error = self.validate_plan_with_pddl(problem_code, response_json)

                    if is_valid:
                        self.current_plan = response_json.get("plan")
                        if not self.current_plan:
                            error_msg = "The generated plan is empty."
                            current_prompt = f"{base_prompt}\n\n⚠️ ERROR: You have generated an empty plan. Please try again."
                            continue

                        # SUCCESS!
                        planning_succeded = True
                        break  # Exit for loop
                    else:
                        logger.warning(f"Invalid PDDL plan: {validation_error}")
                        # Feedback for next attempt: Tell VLM why it failed
                        current_prompt = (
                            f"{base_prompt}\n\n"
                            f"⚠️ PDDL VALIDATION ERROR: The plan you generated is not logically valid.\n"
                            f"The validator reports: {validation_error}\n"
                            f"Think carefully about the preconditions and effects. REGENERATE everything (Problem PDDL + New Actions + Plan)."
                        )

                # --- END RETRY LOOP ---

                if not planning_succeded:
                    if send_prompt == 'y':
                        logger.error("Unable to generate a valid plan after several attempts. Aborting.")
                    task_completed = True
                    continue

                logger.info(f"Plan received with {len(self.current_plan)} steps.")

                """ 
                # For advanced grasping
                bbox_info = self.create_scene_context_string(self.robot.perception.get_collision_objects() ,self.visual_id_to_moveit_map)
                full_prompt = bbox_info + "\n\n" + prompt_di_pianificazione
                full_prompt = prompt_di_pianificazione """

                # Scene_mapping
                scene_mapping = response_json.get("scene_mapping")

                # Inverse Map: Semantic Name -> Visual ID
                semantic_to_visual_id = {v: k for k, v in scene_mapping.items()}
                # Complete Map: Semantic Name -> Real Robot ID
                semantic_to_real_id = {}
                for semantic_name, visual_id in semantic_to_visual_id.items():
                    if visual_id in self.visual_id_to_moveit_map:
                        real_id = self.visual_id_to_moveit_map[visual_id]
                        semantic_to_real_id[semantic_name] = real_id
                    else:
                        logger.warn(f"Annotation ID '{visual_id}' not found in map 'visual_id_to_moveit_map'.")
                        task_completed = True  # Exit loop
                        continue

            # --- 2. EXECUTION PHASE (executes next plan step) ---
            # Take next step from list
            step = self.current_plan.pop(0)  # Extracts first element, shortening list

            skill_name = step.get("skill")
            semantic_args = step.get("arguments", [])

            logger.info(f"Executing Step: {skill_name}({', '.join(semantic_args)})")
            # logger.debug(f"  - Model reasoning: {step.get('reasoning')}")

            # Argument translation: from Semantic to Real Robot ID
            real_args = []
            for arg in semantic_args:
                if arg in semantic_to_real_id:
                    # Case 1: it is a mapped object
                    real_args.append(semantic_to_real_id[arg])
                else:
                    # Case 2: it is a static or unmapped object ("table")
                    # Keep semantic name as string. Specific action will decide if it ignores it.
                    # TODO: find way to read table name from "constants" section of domain.pddl
                    if arg != "table":
                        logger.warn(f"Semantic argument: '{arg}' not found in map 'semantic_to_real_id'.")
                    real_args.append(arg)

            # --- 3. ACTION PROCESSING ---
            task_completed, force_replan = self._process_skill(skill_name, real_args)
            if force_replan:
                logger.info("Failure or ambiguity detected. Abandoning current plan and replanning.")
                self.current_plan = []  # Empty plan to trigger planning logic on next loop
                continue

    def _process_skill(self, skill_name, arguments):
        """
        Handles execution of a skill (or meta-skill like 'done'/'ask'),
        including calling the robot and updating feedback.

        Returns:
            tuple: (task_completed: bool, force_replan: bool)
                    - task_completed: True if task finished (success or impossible).
                    - force_replan: True if action failed and replanning is needed.
        """
        robot = self.robot

        # CASE 1: Task terminated (success or failure declared by model)
        if skill_name == "done":
            logger.info(f"✅ Task completed (obtained 'done' from planner).")
            return True, False

        # CASE 2: Model needs to ask a question (Ambiguity)
        elif skill_name == "ask_for_clarification":
            robot_question = arguments[0] if arguments else "I didn't understand, can you specify?"
            logger.info(f"🤖 ROBOT ASKS: {robot_question}")

            # Get response from user
            risposta_utente = input("Your answer: ")

            self.last_action_feedback = f"""
                    I asked: '{robot_question}'. User answered: '{risposta_utente}'.
                    """
            return False, True

        # CASE 3: Physical Execution
        elif skill_name in robot.available_skills:
            # TODO: PDDL requires two arguments for pick, while we use only one
            if skill_name == "pick":
                arguments = arguments[:-1]
            skill_to_call = robot.available_skills[skill_name]
            success_detector = robot.available_detectors[skill_name]
            try:
                execution_success = skill_to_call(*arguments)

                # AUTO-FEEDBACK (Success Detector)
                if execution_success:
                    # Check if skill succeeded *for real*
                    real_success, feedback_msg = success_detector(*arguments)

                    if real_success:
                        self.last_action_feedback = f"Action '{skill_name}' executed with success."
                        logger.info(f"  - {self.last_action_feedback}")
                        return False, False
                    else:
                        self.last_action_feedback = f"Action '{skill_name}' failed: {feedback_msg}"
                        logger.warning(f"  - {self.last_action_feedback} -> Requesting replanning.")
                        return False, True
                else:
                    self.last_action_feedback = f"ACTION FAILED (Execution): Skill '{skill_name}' reported failure."
                    logger.warning(f"  - {self.last_action_feedback} -> Requesting replanning.")
                    return False, True

            except Exception as e:
                logger.exception(f"❌ Critical error executing '{skill_name}'.")
                self.last_action_feedback = f"Critical error during execution: {e}"
                return False, True
        else:
            logger.error(f"Error: Gemini chose an unmapped skill: {skill_name}")
            self.last_action_feedback = f"Planning error: chosen skill '{skill_name}' nonexistent."
            return False, True

    def create_scene_context_string(self, collision_objects, visual_id_map):
        """
        Creates a descriptive string with physical dimensions.
        visual_id_map: { "1": "box_0", "2": "box_1" }
        """
        context_lines = ["\n*** DETECTED PHYSICAL DATA (BBOX) ***"]

        from shape_msgs.msg import SolidPrimitive

        for viz_id, moveit_id in visual_id_map.items():
            # Find corresponding object
            obj = next((co for co in collision_objects if co.id == moveit_id), None)
            if not obj: continue

            # Assume BOX for simplicity (MoveIt uses primitives)
            if obj.primitives and obj.primitives[0].type == SolidPrimitive.BOX:
                dims = obj.primitives[0].dimensions  # [x, y, z]
                # Convert to cm for readability or leave meters
                l = dims[SolidPrimitive.BOX_X]
                w = dims[SolidPrimitive.BOX_Y]
                h = dims[SolidPrimitive.BOX_Z]  # HEIGHT! The critical data.

                line = f"- Object ID {viz_id}: Dimensions [LxWxH] = {l:.3f} x {w:.3f} x {h:.3f} m."
                context_lines.append(line)

        return "\n".join(context_lines)

    def validate_plan_with_pddl(self, pddl_problem_string, plan_json):
        # ---------------------------------------------------------
        # 1. LOAD PDDL ENVIRONMENT
        # ---------------------------------------------------------
        reader = PDDLReader()

        # UP usually wants physical files. Create temp file for problem.
        # (Alternatively handle streams, but this is safer for C++ parsers)
        # ADDED UTF-8 encoding for parsing problems (due to Italian chars in testing complaining about "è")
        with tempfile.NamedTemporaryFile(mode='w+', delete=False, suffix='.pddl', encoding='utf-8') as tmp:
            tmp.write(pddl_problem_string)
            tmp_problem_path = tmp.name

        try:
            # Parsing: Here UP understands which objects and actions exist
            up_problem = reader.parse_problem(self.pddl_domain_path, tmp_problem_path)
        finally:
            os.remove(tmp_problem_path)  # Cleanup

        # ---------------------------------------------------------
        # 2. CONVERSION: JSON -> SequentialPlan
        # ---------------------------------------------------------
        plan_actions = []

        # FOR DEBUG: Force validation failure
        # plan_json["plan"] = list(reversed(plan_json["plan"]))
        for step in plan_json["plan"]:
            action_name = step["skill"]
            params_names = step["arguments"]

            # A. Filter "meta" actions not in PDDL
            if action_name.lower() == "done":
                continue

            # B. Retrieve Action object from problem
            try:
                # Note: UP is case-sensitive by default.
                # If PDDL has "pick" and JSON "Pick", use .lower() if needed
                up_action = up_problem.action(action_name)
            except KeyError:
                print(f"ERROR: Action '{action_name}' does not exist in domain!")
                break

            # C. Retrieve Parameter objects
            up_params = []
            try:
                for p_name in params_names:
                    # Search object in problem (e.g. search physical object "Cuboid")
                    up_obj = up_problem.object(p_name)
                    up_params.append(up_obj)
            except KeyError as e:
                print(f"ERROR: Object '{e}' required by action '{action_name}' does not exist in problem!")
                break

            # D. Create instance and add to list
            # ActionInstance wants (Action, Tuple[Object, ...])
            plan_actions.append(ActionInstance(up_action, tuple(up_params)))

        # Creation of final object accepted by validator
        up_plan = SequentialPlan(plan_actions, up_problem.environment)

        # ---------------------------------------------------------
        # 3. VALIDATION
        # ---------------------------------------------------------
        with PlanValidator(name="sequential_plan_validator") as validator:
            result = validator.validate(up_problem, up_plan)

            if result.status.name == 'VALID':
                print("✅ [PDDL_PLAN_VALIDATOR] SUCCESS: The plan is valid and executable.")
                return True, None
            else:
                print("❌ FAILURE: The plan is not valid.")
                # Extract error message from validator logs
                # Usually in result.log_messages
                error_details = str(result.log_messages)
                print("Reason:", result.log_messages)

                # If you want something cleaner, parse result.metric_evaluations or similar
                return False, error_details

    def process_vlm_response(self, response_json):
        # 1. Need to update domain?
        new_actions = response_json.get("new_actions", [])

        current_domain_path = self.pddl_domain_path

        if new_actions:
            print(f"⚠️ VLM requested {len(new_actions)} new actions. Updating domain...")

            # Create copy of current domain to avoid dirtying original if something goes wrong
            updated_domain_path = "domain_updated.pddl"
            shutil.copy(current_domain_path, updated_domain_path)

            # Helper function to inject PDDL text into file
            self.inject_actions_into_domain(updated_domain_path, new_actions)

            # Use new path for rest of execution
            current_domain_path = updated_domain_path

        # print(current_domain_path) # DEBUG
        self.pddl_domain_path = current_domain_path
        # 2. Now proceed with plan validation using domain (old or updated)
        # self.validate_plan_with_pddl(response_json["plan"], current_domain_path)

    def inject_actions_into_domain(self, domain_path, new_actions_list):
        with open(domain_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Find last closing parenthesis
        last_bracket_index = content.rfind(')')

        if last_bracket_index == -1:
            raise ValueError("Malformed PDDL file: closing parenthesis not found.")

        # Construct text block to insert
        # Add newline for cleanliness
        injected_code = "\n\n  ;; --- DYNAMICALLY GENERATED ACTIONS BY VLM ---\n"
        for item in new_actions_list:
            # CASE 1: VLM returns structured object (what it is doing now)
            if isinstance(item, dict):
                # Search PDDL definition in correct key
                # Use .get() for safety, handling potential key name variants
                action_code = item.get("pddl") or item.get("pddl_definition") or item.get("code") or item.get(
                    "definition")

                if not action_code:
                    print(f"⚠️ WARNING: Found action object without PDDL definition: {item}")
                    continue

            # CASE 2: VLM returns string directly (backward compatibility)
            elif isinstance(item, str):
                action_code = item
            else:
                print(f"⚠️ Unsupported type in action list: {type(item)}")
                continue

            # Add code to buffer
            injected_code += action_code + "\n"

        # Reconstruct content: Everything before last bracket + New Actions + Last bracket
        new_content = content[:last_bracket_index] + injected_code + content[last_bracket_index:]

        with open(domain_path, 'w', encoding='utf-8') as f:
            f.write(new_content)