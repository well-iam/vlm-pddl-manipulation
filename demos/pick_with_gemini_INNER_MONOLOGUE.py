from tesi_gemini_robotics.coppeliasim_setup import connect_to_sim
from tesi_gemini_robotics.robot_controller import RobotController
from tesi_gemini_robotics.llm_interface import GeminiClient
from tesi_gemini_robotics.utils import get_image_of_scene, get_top_level_objects, convert_object_names_to_handles, simulation_stepper
from tesi_gemini_robotics.prompt_templates import SYSTEM_INSTRUCTION_COLLABORATIVE_PLANNER, SYSTEM_INSTRUCTION_PLANNER
import time
import logging
from tesi_gemini_robotics.logger_config import setup_logging
from tesi_gemini_robotics.task_executor import TaskExecutor
import threading

logger = logging.getLogger(__name__)
SYSTEM_INSTRUCTION_TEMPLATE = SYSTEM_INSTRUCTION_PLANNER

def main():

    setup_logging(logging.DEBUG)

    # INIZIALIZZAZIONE SIMULAZIONE
    client, sim, simIK, simOMPL = connect_to_sim()
    if not client: return

    # INIZIALIZZAZIONE DI CLASSI ROBOT E GEMINI_CLIENT
    try:
        # Crea l'istanza del RobotController (che fa tutto il setup pesante)
        robot = RobotController(client, sim, simIK, simOMPL, robot_name='Franka')

        # --- 1. Inizializza il client Gemini ---
        # Puoi scegliere il modello qui
        gemini_planner = GeminiClient(model_name='models/gemini-robotics-er-1.5-preview', system_instruction_template=SYSTEM_INSTRUCTION_TEMPLATE)

        # (Opzionale) Potresti creare un secondo client per un altro modello
        # gemini_describer = GeminiClient(model_name='models/gemini-1.5-flash-latest')
        task_executor = TaskExecutor(sim, gemini_planner, robot)

    except Exception as e:
        print("Impossibile inizializzare il client Gemini. Termino.")
        print(e)
        return

    # --- Inizio Simulazione ---
    sim.startSimulation()
    time.sleep(0.2) #Serve per dare tempo ai buffer del sensore di immagini di riempirsi
    sim.setBoolProperty(sim.getObject('/Red_Cuboid'), 'dynamic', True, 0)
    # sim.setBoolProperty(sim.getObject('/Red_Cuboid'), 'respondable', True, 0)
    sim.setBoolProperty(sim.getObject('/Green_Cuboid'), 'dynamic', True, 0)
    # sim.setBoolProperty(sim.getObject('/Green_Cuboid'), 'respondable', True, 0)
    sim.setBoolProperty(sim.getObject('/Blue_Cuboid'), 'dynamic', True, 0)

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

    # Ottieni la lista dei nomi degli oggetti dal simulatore
    objects_to_ignore = ['Floor', 'Franka', 'FakeFranka', 'Blue_Wall']
    available_objects_name, available_objects_handle = get_top_level_objects(sim, ignore_list=objects_to_ignore)
    identified_objects_handles = available_objects_handle

    # # Acquisizione immagine della scena
    # immagine_scena = get_image_of_scene(sim)
    #
    # # Gemini identifica gli oggetti con i loro nomi in simulazione
    # identified_objects_names = gemini_planner.get_object_list_from_image(immagine_scena, available_objects_name)
    # if not identified_objects_names:
    #     print("Nessun oggetto identificato. Termino il programma.")
    #     sim.stopSimulation()
    #     return
    # identified_objects_handles = convert_object_names_to_handles(sim, identified_objects_names)

    # OUTER LOOP (Livello Utente)
    while True:
        # INPUT UTENTE
        comando_utente = input("Inserisci comando utente ('q' per terminare): ")
        if comando_utente in ['q', 'Q']:
            break

        # Passa il controllo all'Inner Loop
        task_executor.execute(comando_utente, identified_objects_handles)
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

