import time
import logging
import sys
from pathlib import Path
from tesi_gemini_robotics import setup_logging
from tesi_gemini_robotics import connect_to_sim
from tesi_gemini_robotics import GeminiClient
from tesi_gemini_robotics import TaskExecutor
from tesi_gemini_robotics import system_instruction_dict
from tesi_gemini_robotics.implementations.coppelia.coppeliasim_robot import CoppeliaSimRobot

logger = logging.getLogger(__name__)
# Choose between: pddl, no_pddl_collaborative
SYSTEM_INSTRUCTION_NAME = "pddl_dynamic_actions"

def main():

    setup_logging(logging.DEBUG)
    pddl_domain_path = ""
    if SYSTEM_INSTRUCTION_NAME.startswith("pddl"):
        script_dir = Path(__file__).parent.resolve()
        # Read the content of the domain.pddl file
        pddl_domain_path = script_dir.parent / "src" / "tesi_gemini_robotics" / "core" / "domain.pddl"
        try:
            with open(pddl_domain_path, "r") as f:
                pddl_domain_content = f.read()
                system_instruction = system_instruction_dict[SYSTEM_INSTRUCTION_NAME].format(pddl_domain_content=pddl_domain_content)
        except FileNotFoundError:
            logger.error(f"Error: The file {pddl_domain_path} was not found.")
            sys.exit(1)
    else:
        system_instruction = system_instruction_dict[SYSTEM_INSTRUCTION_NAME]

    # INITIALIZATION OF ROBOT AND GEMINI_CLIENT CLASSES
    try:
        
        # SIMULATION INITIALIZATION
        client, sim, simIK, simOMPL = connect_to_sim()
        if not client: return
        # Create the RobotController instance
        robot = CoppeliaSimRobot(client, sim, simIK, simOMPL, robot_name='Franka')
        # --- Start Simulation ---
        sim.startSimulation()
        time.sleep(0.2)  # Needed to give time for image sensor buffers to fill


        # --- 1. Initialize Gemini client ---
        gemini_planner = GeminiClient(model_name='models/gemini-2.5-flash', system_instruction=system_instruction)

        task_executor = TaskExecutor(gemini_planner, robot, pddl_domain_path)

    except Exception as e:
        print("Unable to initialize Gemini client. Terminating.")
        print(e)
        return

    # OUTER LOOP (User Level)
    while True:
        # USER INPUT
        user_command = input("Enter user command ('q' to exit): ")
        if user_command in ['q', 'Q']:
            break

        if user_command=="open":
            robot.gripper.open()
        elif user_command=="close":
            robot.gripper.close()
        elif user_command=="nudge":
            robot.perform_nudge("blue_cuboid")

        # Pass control to the Inner Loop
        task_executor.execute(user_command)
        print("======= AUTONOMOUS TASK FINISHED. Waiting for a new goal. =======")

    # --- End Simulation ---
    input('Press ENTER to end the simulation.')
    sim.stopSimulation()
    print("Simulation ended.")

if __name__ == '__main__':
    main()