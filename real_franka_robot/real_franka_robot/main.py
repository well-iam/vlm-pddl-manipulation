import rclpy
import time
import logging
import sys
from pathlib import Path
from .vlm_agent import setup_logging
from .vlm_agent import GeminiClient
from .vlm_agent import TaskExecutor
from .vlm_agent import system_instruction_dict
from .vlm_agent import FrankaRobot
from dotenv import load_dotenv

load_dotenv()
logger = logging.getLogger(__name__)
# Choose between: pddl, no_pddl_collaborative
SYSTEM_INSTRUCTION_NAME = "pddl_dynamic_actions"

def main():
    setup_logging(logging.DEBUG)
    if SYSTEM_INSTRUCTION_NAME.startswith("pddl"):
        script_dir = Path(__file__).parent.resolve()
        # Read the content of the domain.pddl file
        pddl_domain_path = script_dir / "vlm_agent" / "core" / "domain.pddl"
        try:
            with open(pddl_domain_path, "r") as f:
                pddl_domain_content = f.read()
                system_instruction = system_instruction_dict[SYSTEM_INSTRUCTION_NAME].format(
                    pddl_domain_content=pddl_domain_content)
        except FileNotFoundError:
            logger.error(f"Error: The file {pddl_domain_path} was not found.")
            sys.exit(1)
    else:
        system_instruction = system_instruction_dict[SYSTEM_INSTRUCTION_NAME]


    # INITIALIZATION OF ROBOT AND GEMINI_CLIENT CLASSES
    try:
        rclpy.init(signal_handler_options=rclpy.SignalHandlerOptions.NO)
        robot = FrankaRobot()

        # --- Initialize the Gemini client ---
        # You can choose the model here: gemini-robotics-er-1.5-preview / models/gemini-2.5-flash
        gemini_planner = GeminiClient(model_name='gemini-robotics-er-1.5-preview',
                                      system_instruction=system_instruction)

        task_executor = TaskExecutor(gemini_planner, robot, pddl_domain_path)

    except Exception as e:
        print("Unable to initialize Gemini client. Terminating.")
        print(e)
        return

    try:
        # OUTER LOOP (User Level)
        while True:
            # USER INPUT
            high_level_goal = input("Enter user command ('q' to exit): ")
            if high_level_goal in ['q', 'Q']:
                break

            if high_level_goal == "open":
                robot.gripper.open()
            elif high_level_goal == "close":
                robot.gripper.close()

            # Pass control to the Inner Loop
            task_executor.execute(high_level_goal)
            print("======= AUTONOMOUS TASK COMPLETED. Awaiting new objective. =======")

    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.exception("Exception detected during execution: ")
    finally:
        if input("Do you want to reset the robot (y/n)?: ") == 'y':
            robot.shutdown()
        print("Shutdown...")
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()