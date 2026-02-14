import logging
import json
import re
import os
import tempfile
import shutil
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

    def execute(self, goal, strategy="planning"):
        """
        Unified method to launch the task with the chosen strategy.
        """
        if strategy == "reactive":
            logger.info("Starting execution in REACTIVE mode (Inner Monologue)...")
            return self.execute_task_step_by_step(goal)

        elif strategy == "planning":
            logger.info("Starting execution in PLANNING mode (Plan-and-Execute)...")
            return self.execute_plan(goal)

        else:
            raise ValueError(f"Strategy '{strategy}' not supported.")

    def _update_perception(self):
        """
        Internal method to update perception and save the state.
        Returns the JSON string ready for the prompt.
        """
        self.robot.perception.perceive_scene(self.gemini_planner, use_vision=False)
        # 1. Get the dictionary (saved for internal use)
        # Note: get_world_state_data returns a DICTIONARY, not a string
        self.world_state = self.robot.perception.get_world_state_data()

        # 2. Convert to JSON (only for the prompt)
        return json.dumps(self.world_state, indent=2)

    def _process_skill(self, skill_name, arguments):
        """
        Manages the execution of a skill (or meta-skill like 'done'/'ask'),
        including the robot call and feedback update.

        Returns:
            tuple: (task_completed: bool, force_replan: bool)
                   - task_completed: True if the task is finished (success or impossible).
                   - force_replan: True if the action failed and replanning is needed.
        """
        robot = self.robot

        # CASE 1: The task is finished (success or failure declared by the model)
        if skill_name == "done":
            logger.info(f"✅ Task completed (received 'done' from planner).")
            return True, False

        # CASE 2: The model needs to ask a question (Ambiguity)
        elif skill_name == "ask_for_clarification":
            robot_question = arguments[0] if arguments else "I did not understand, can you specify?"
            logger.info(f"🤖 ROBOT ASKS: {robot_question}")

            # Get response from user
            user_answer = input("Your answer: ")

            self.last_action_feedback = f"""
                    I asked: '{robot_question}'. User has replied: '{user_answer}'.
                    """
            return False, True

        # CASE 3: Physical Execution
        elif skill_name in robot.available_skills or "nudge" in skill_name:
            # TODO: PDDL requires two arguments for pick, while we use only one
            if skill_name == "pick":
                arguments = arguments[:-1]
            elif "nudge" in skill_name:
                arguments = arguments[:1]
                skill_name = "nudge"
            skill_to_call = robot.available_skills[skill_name]
            success_detector = robot.available_detectors[skill_name]
            try:
                execution_success = skill_to_call(*arguments)

                # AUTO-FEEDBACK (Success Detector)
                if execution_success:
                    # Check if skill *really* succeeded
                    real_success, feedback_msg = success_detector(*arguments)

                    if real_success:
                        self.last_action_feedback = f"Action '{skill_name}' executed successfully."
                        logger.info(f"  - {self.last_action_feedback}")
                        return False, False
                    else:
                        self.last_action_feedback = f"Action '{skill_name}' has failed: {feedback_msg}"
                        logger.warning(f"  - {self.last_action_feedback} -> Replanning requested.")
                        return False, True
                else:
                    self.last_action_feedback = f"ACTION FAILED (Execution): skill '{skill_name}' has reported a failure."
                    logger.warning(f"  - {self.last_action_feedback} -> Replanning requested.")
                    return False, True

            except Exception as e:
                logger.exception(f"❌ Critical error in execution of '{skill_name}'.")
                self.last_action_feedback = f"Critical error during execution: {e}"
                return False, True
        else:
            logger.error(f"Error: Gemini chose an unmapped skill: {skill_name}")
            self.last_action_feedback = f"Planning error: chosen skill '{skill_name}' inexistent."
            return False, True

    def execute_task_step_by_step(self, high_level_goal):
        """
        Executes the INNER LOOP (Inner Monologue) for a single objective.
        """
        print(f"\n======= STARTING AUTONOMOUS TASK: '{high_level_goal}' =======")
        gemini_planner = self.gemini_planner
        robot = self.robot

        # Initial state for the loop
        self.last_action_feedback = "No previous action."

        task_completed = False
        step_counter = 0
        while not task_completed and step_counter < self.max_steps:
            step_counter += 1
            print(f"\n--- Turn {step_counter}/{self.max_steps} ---")

            # PERCEPTION
            world_state_json = self._update_perception()

            # --- 3. PLANNING (Build turn prompt) ---
            current_prompt = f"""
            **Current World State:**
            {world_state_json}

            **Previous Action Feedback:**
            {self.last_action_feedback}

            **Final Goal:**
            "{high_level_goal}"

            Generate the JSON for the next action.
            """

            # PLANNING
            gemini_choice = gemini_planner.ask_for_skill_json(current_prompt, list(robot.available_skills.keys()))

            # --- 4. EXECUTION AND FEEDBACK ---
            if not gemini_choice:
                print("Planning failed: Gemini did not provide a valid response.")
                self.last_action_feedback = "Planner error: I did not receive a valid JSON."
                continue

            skill_name = gemini_choice["skill"]
            arguments = gemini_choice["arguments"]
            # --- 6. EXECUTE SKILL ---
            task_completed, force_replan = self._process_skill(skill_name, arguments)

        # --- 8. TERMINATION CONDITION (FAILURE/TIMEOUT) ---
        if task_completed:
            logger.info(f"✅ Task completed.")
            return True
        else:
            print(f"❌ Task failed: Reached maximum limit of {self.max_steps} steps.")
            return False

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
            print("✅ Problem PDDL extracted.")
        else:
            print("⚠️ WARNING: No PDDL block found in response.")

        # 2. JSON Extraction (searches for text between ```json and ```)
        json_pattern = r"```json\s*(.*?)\s*```"
        json_match = re.search(json_pattern, response_text, re.DOTALL)

        plan_json = {}
        if json_match:
            json_str = json_match.group(1).strip()
            try:
                plan_json = json.loads(json_str)
                print("✅ Plan JSON extracted and validated.")
            except json.JSONDecodeError as e:
                print(f"❌ Error parsing JSON: {e}")
        else:
            # Fallback: sometimes Gemini responds with naked JSON without backticks
            try:
                # Desperate attempt: find first open brace and last close brace
                start = response_text.find('{')
                end = response_text.rfind('}') + 1
                if start != -1 and end != -1:
                    plan_json = json.loads(response_text[start:end])
                    print("⚠️ JSON extracted without markdown (fallback).")
            except:
                print("❌ Impossible to find valid JSON.")

        return problem_pddl_content, plan_json

    def execute_plan(self, high_level_goal):
        """
        Executes an autonomous task:
        1. Plans a complete plan.
        2. Executes the plan step-by-step.
        3. If a step fails, stops and replans.
        """
        logger.info(f"======= STARTING AUTONOMOUS TASK: '{high_level_goal}' =======")
        gemini_client = self.gemini_planner

        # --- EXECUTION CYCLE STATE ---
        self.current_plan = []  # The list of steps to execute
        self.last_action_feedback = "No previous action. Questo è il primo passo."
        task_completed = False
        # Parameters for retry loop
        MAX_RETRIES = 3
        # Start chat. The system instruction will be formatted and sent in the loop
        gemini_client.start_chat()  # Resets history

        while not task_completed:
            # --- 1. PLANNING OR REPLANNING PHASE ---
            if not self.current_plan:  # If we don't have a plan, create one
                logger.info("No active plan. Generating a new plan...")

                # 1a. Perception
                world_state_json = self._update_perception()

                # 1c. Plan Request
                base_prompt = f"""
                **Current World State:**
                {world_state_json}

                **Previous Action Feedback:**
                {self.last_action_feedback}

                **Final Goal:**
                "{high_level_goal}"

                Generate the complete plan JSON for reaching the goal.
                """

                # --- START RETRY LOOP ---
                current_prompt = base_prompt
                planning_success = False

                for retry in range(1, MAX_RETRIES + 1):
                    logger.info(f"--- Planning Attempt {retry}/{MAX_RETRIES} ---")

                    # Check API call (optional, to save tokens during debug)
                    send_prompt = input("Do you want to send the prompt to Gemini (CALLS ARE LIMITED!) (y/n): ")
                    if send_prompt != "y": break

                    gemini_answer = gemini_client.send_chat_message(current_prompt)

                    # 1. Syntactic Parsing (Broken JSON/PDDL)
                    try:
                        problem_code, response_json = self._parse_pddl_gemini_response(gemini_answer.text)
                        self.process_vlm_response(response_json)  # updates path to domain.pddl if requested by vlm
                    except Exception as e:
                        error_msg = f"Syntax error in JSON/PDDL: {str(e)}"
                        logger.warning(error_msg)
                        # Feedback for next attempt
                        current_prompt = f"{base_prompt}\n\n⚠️ CRITICAL ERROR: Your previous answer had an invalid syntax. Correct this error:\n{error_msg}"
                        continue  # Retry

                    # 2. Logical Validation (PDDL Validator)
                    # Note: Modify validate_plan_with_pddl to return (bool, error_string)
                    # 1. Extract the plan list safely to avoid repeating .get()
                    plan_list = response_json.get("plan", [])

                    if not plan_list:
                        current_prompt = f"{base_prompt}\n\n⚠️ ERROR: You generated an empty plan. Retry."
                        continue

                    # 2. Check the first skill WITHOUT popping it
                    first_skill = plan_list[0].get("skill")

                    if first_skill == "ask_for_clarification":
                        # CASE A: Clarification -> Skip PDDL validation, accept plan immediately
                        logger.debug("Model asked for clarification. Skipping PDDL validation.")
                        self.current_plan = plan_list
                        planning_success = True
                        break  # Exit retry loop
                    else:
                        # CASE B: Physical Action -> Perform PDDL Validation
                        is_valid, validation_error = self.validate_plan_with_pddl(problem_code, response_json)

                        if is_valid:
                            self.current_plan = plan_list
                            planning_success = True
                            break  # Exit retry loop
                        else:
                            logger.warning(f"Invalid PDDL plan: {validation_error}")
                            # Feedback for next attempt
                            current_prompt = (
                                f"{base_prompt}\n\n"
                                f"⚠️ PDDL VALIDATION ERROR: The plan you generated is not logically valid.\n"
                                f"The validator reports: {validation_error}\n"
                                f"Think carefully about the preconditions and effects. REGENERATE everything (Problem PDDL + New Actions + plan)."
                            )

                # --- END RETRY LOOP ---

                if not planning_success:
                    logger.error("Impossible to generate a valid plan after several attempts. Stopping.")
                    task_completed = True
                    continue

                logger.info(f"Valid plan received with {len(self.current_plan)} steps.")

            # --- 2. EXECUTION PHASE (executes next step of plan) ---
            # Get next step from list
            step = self.current_plan.pop(0)  # Extracts first element, shortening the list

            skill_name = step.get("skill")
            arguments = step.get("arguments", [])

            logger.info(f"Execution Step: {skill_name}({', '.join(arguments)})")
            logger.debug(f"  - Model reasoning: {step.get('reasoning')}")

            # --- 3. ACTION PROCESSING ---
            task_completed, force_replan = self._process_skill(skill_name, arguments)
            if force_replan:
                logger.info("Failure or ambiguity detected. Abandoning current plan and replanning.")
                self.current_plan = []  # Empty plan to trigger planning logic on next turn
                continue

    def validate_plan_with_pddl(self, pddl_problem_string, plan_json):
        # ---------------------------------------------------------
        # 1. DATA INPUT (Simulation)
        # ---------------------------------------------------------
        # path_domain = "C:/Users/William/Desktop/Tesi_Gemini_Robotics/src/tesi_gemini_robotics/core/domain.pddl"  # Ensure file exists

        # Problem string generated by VLM
        # vlm_problem_string = """(define (problem p) (:domain d) ... )"""

        # JSON generated by VLM
        # plan_json

        # ---------------------------------------------------------
        # 2. LOAD PDDL ENVIRONMENT
        # ---------------------------------------------------------
        reader = PDDLReader()

        # UP usually wants physical files. Create a temporary file for the problem.
        # (Alternatively one could manage streams, but this is safer for C++ parsers)
        # ADDED UTF-8 encoding for parsing problems (due to this testing phase being in Italian and complaining about "è")
        with tempfile.NamedTemporaryFile(mode='w+', delete=False, suffix='.pddl', encoding='utf-8') as tmp:
            tmp.write(pddl_problem_string)
            tmp_problem_path = tmp.name

        try:
            # Parsing: Here UP understands which objects and actions exist
            up_problem = reader.parse_problem(self.pddl_domain_path, tmp_problem_path)
        finally:
            os.remove(tmp_problem_path)  # Cleanup

        # ---------------------------------------------------------
        # 3. CONVERSION: JSON -> SequentialPlan
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
                # If PDDL has "pick" and JSON "Pick", use .lower() if necessary
                up_action = up_problem.action(action_name)
            except KeyError:
                print(f"ERROR: Action '{action_name}' does not exist in domain!")
                break

            # C. Retrieve Parameters objects
            up_params = []
            try:
                for p_name in params_names:
                    # Search object in problem (e.g., search physical object "Cuboid")
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
        # 4. VALIDATION
        # ---------------------------------------------------------
        with PlanValidator(name="sequential_plan_validator") as validator:
            result = validator.validate(up_problem, up_plan)

            if result.status.name == 'VALID':
                print("✅ SUCCESS: Plan is valid and executable.")
                return True, None
            else:
                print("❌ FAILURE: Plan is not valid.")
                # Extract error message from validator logs
                # Usually in result.log_messages
                error_details = str(result.log_messages)
                print("Reason:", result.log_messages)

                # If you want something cleaner, parse result.metric_evaluations or similar
                return False, error_details

    def process_vlm_response(self, response_json):

        # 1. Is there a need to update the domain?
        new_actions = response_json.get("new_pddl_actions", [])

        current_domain_path = self.pddl_domain_path

        if new_actions:
            print(f"⚠️ VLM requested {len(new_actions)} new actions. Updating domain...")

            # Create a copy of current domain to avoid corrupting original if something goes wrong
            updated_domain_path = "domain_updated.pddl"
            shutil.copy(current_domain_path, updated_domain_path)

            # Helper function to inject PDDL text into the file
            self.inject_actions_into_domain(updated_domain_path, new_actions)

            # Use new path for rest of execution
            current_domain_path = updated_domain_path

        print(current_domain_path)
        self.pddl_domain_path = current_domain_path
        # 2. Now proceed with plan validation using the domain (old or updated)
        # self.validate_plan_with_pddl(response_json["plan"], current_domain_path)

    def inject_actions_into_domain(self, domain_path, new_actions_list):
        with open(domain_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Find last closing parenthesis
        last_bracket_index = content.rfind(')')

        if last_bracket_index == -1:
            raise ValueError("Malformed PDDL file: closing parenthesis not found.")

        # Build code block to insert
        # Add newline for cleanliness
        injected_code = "\n\n  ;; --- DYNAMICALLY GENERATED ACTIONS BY VLM ---\n"
        for item in new_actions_list:
            # CASE 1: VLM returns a structured object (what it is doing now)
            if isinstance(item, dict):
                # Look for PDDL definition in correct key
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