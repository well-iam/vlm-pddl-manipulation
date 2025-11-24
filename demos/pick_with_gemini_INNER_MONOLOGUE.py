import time
import logging
from tesi_gemini_robotics import setup_logging
from tesi_gemini_robotics import connect_to_sim
from tesi_gemini_robotics import GeminiClient
from tesi_gemini_robotics import TaskExecutor
from tesi_gemini_robotics import SYSTEM_INSTRUCTION_PLANNER, SYSTEM_INSTRUCTION_COLLABORATIVE_PLANNER

# --- IL "SWITCH" MAGICO ---
USE_SIMULATION = True

if USE_SIMULATION:
    from tesi_gemini_robotics.implementations.coppelia.coppeliasim_robot import CoppeliaSimRobot

else:
    from tesi_gemini_robotics.implementations.franka.franka_robot import FrankaRobot

# --------------------------

logger = logging.getLogger(__name__)
SYSTEM_INSTRUCTION_TEMPLATE = SYSTEM_INSTRUCTION_COLLABORATIVE_PLANNER

def main():

    setup_logging(logging.DEBUG)

    # INIZIALIZZAZIONE DI CLASSI ROBOT E GEMINI_CLIENT
    try:
        # Crea l'istanza del RobotController (che fa tutto il setup pesante)
        if USE_SIMULATION:
            # INIZIALIZZAZIONE SIMULAZIONE
            client, sim, simIK, simOMPL = connect_to_sim()
            if not client: return

            robot = CoppeliaSimRobot(client, sim, simIK, simOMPL, robot_name='Franka')
            # --- Inizio Simulazione ---
            sim.startSimulation()
            time.sleep(0.2)  # Serve per dare tempo ai buffer del sensore di immagini di riempirsi
        else:
            # robot = FrankaRobot(robot_ip="192.168.1.100")
            pass

        # --- 1. Inizializza il client Gemini ---
        # Puoi scegliere il modello qui
        gemini_planner = GeminiClient(model_name='models/gemini-robotics-er-1.5-preview', system_instruction_template=SYSTEM_INSTRUCTION_TEMPLATE)

        # (Opzionale) Potresti creare un secondo client per un altro modello
        # gemini_describer = GeminiClient(model_name='models/gemini-1.5-flash-latest')
        task_executor = TaskExecutor(gemini_planner, robot)

    except Exception as e:
        print("Impossibile inizializzare il client Gemini. Termino.")
        print(e)
        return

    # # --- AVVIA IL BATTITO CARDIACO ---
    # stop_event = threading.Event()
    # sim_thread = threading.Thread(
    #     target=simulation_stepper,
    #     args=(client, stop_event),
    #     daemon=True  # Il thread si chiuderà automaticamente se il main crasha
    # )
    # sim_thread.start()
    # # Dai al thread un istante per eseguire il primo 'step'
    # time.sleep(0.1)

    # OUTER LOOP (Livello Utente)
    while True:
        # INPUT UTENTE
        comando_utente = input("Inserisci comando utente ('q' per terminare): ")
        if comando_utente in ['q', 'Q']:
            break

        if comando_utente=="open":
            robot.gripper.open()
        elif comando_utente=="close":
            robot.gripper.close()

        # Passa il controllo all'Inner Loop
        task_executor.execute(comando_utente)
        print("======= TASK AUTONOMO TERMINATO. In attesa di un nuovo obiettivo. =======")

    # # 1. Ferma il battito cardiaco
    # stop_event.set()
    # print("Segnale di stop inviato al thread stepper...")
    # sim_thread.join()  # Attende che il thread termini
    # print("Thread stepper fermato.")

    # --- Fine Simulazione ---
    input('Premi INVIO per terminare la simulazione.')
    sim.stopSimulation()
    print("Simulazione terminata.")

if __name__ == '__main__':
    main()

