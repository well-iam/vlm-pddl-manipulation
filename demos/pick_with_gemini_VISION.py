from tesi_gemini_robotics.coppeliasim_setup import connect_to_sim
from tesi_gemini_robotics.robot_controller import RobotController
from tesi_gemini_robotics.llm_interface import GeminiClient
from tesi_gemini_robotics.utils import get_image_of_scene, get_top_level_interactive_objects
import time

# --- 2. FUNZIONI DI CONTROLLO E MOVIMENTO ---

# def control_gripper(sim, handles, action='close'):
#     """Controlla l'apertura o la chiusura del gripper."""
#     target_pos = 0.0 if action == 'close' else 0.04  # 0 = chiuso, 0.04 = aperto
#     print(f"Comando gripper: {'chiusura' if action == 'close' else 'apertura'} (target: {target_pos} m)")
#
#     for joint_handle in handles['gripper_joints']:
#         sim.setJointTargetPosition(joint_handle, target_pos)
#
#     # Attendi che il movimento del gripper si completi
#     time.sleep(1.5)

# --- 4. FUNZIONE PRINCIPALE (MAIN) ---

def main():
    """Orchestra l'intera operazione di pick and hold."""
    client, sim, simIK, simOMPL = connect_to_sim()
    if not client: return

    try:

        # Crea l'istanza del RobotController (che fa tutto il setup pesante)
        robot = RobotController(client, sim, simIK, simOMPL, robot_name='Franka')

        # --- 1. Inizializza il client Gemini ---
        # Puoi scegliere il modello qui
        gemini_planner = GeminiClient(model_name='models/gemini-robotics-er-1.5-preview')

        # (Opzionale) Potresti creare un secondo client per un altro modello
        # gemini_describer = GeminiClient(model_name='models/gemini-1.5-flash-latest')

    except Exception as e:
        print("Impossibile inizializzare il client Gemini. Termino.")
        return  # Esce se non può creare il client




    # --- Inizio Simulazione ---
    sim.startSimulation()
    time.sleep(0.2) #Serve per dare tempo ai buffer del sensore di immagini di riempirsi

    # Ottieni la lista di nomi dal simulatore PRIMA di tutto
    objects_to_ignore = ['Floor', 'Franka', 'FakeFranka']
    available_objects = get_top_level_interactive_objects(sim, ignore_list=objects_to_ignore)

    # Acquisizione immagine
    immagine_scena = get_image_of_scene(sim)

    # 1. Gemini identifica gli oggetti
    # Passa la lista a Gemini
    identified_objects = gemini_planner.get_object_list_from_image(immagine_scena, available_objects)
    if not identified_objects:
        print("Nessun oggetto identificato. Termino il programma.")
        sim.stopSimulation()
        return

    # Informazioni sulla scena (da CoppeliaSim o altro)
    #immagine_scena = get_image_of_scene(sim)

    obiettivo_utente = "Afferra il cubo."

    # Usa il metodo della classe per ottenere la scelta
    scelta_gemini = gemini_planner.get_next_skill_from_image(
        immagine_scena,
        obiettivo_utente,
        list(robot.available_skills.keys()),
        identified_objects
    )

    # Esegui l'azione scelta
    if scelta_gemini:
        skill_nome = scelta_gemini["skill_scelta"]
        argomenti = scelta_gemini["argomenti"]

        if skill_nome in robot.available_skills:
            funzione_da_chiamare = robot.available_skills[skill_nome]
            try:
                successo = funzione_da_chiamare(*argomenti)
                # ... (gestisci successo/fallimento) ...
            except TypeError:
                print(f"❌ Errore: numero errato di argomenti ({argomenti}) per la skill '{skill_nome}'.")
            except Exception as e:
                print(f"❌ Errore durante l'esecuzione della skill '{skill_nome}': {e}")
    else:
        print("\nGemini non ha suggerito una skill valida.")

    #object_to_pick = 'Cuboid'
    #success = perform_pick_and_hold(client, sim, simIK, simOMPL, handles, ik_handles, object_to_pick)



    # --- Fine Simulazione ---
    input('Premi INVIO per terminare la simulazione.')
    sim.stopSimulation()
    # (TEMPORANEO) Setto dynamic e respondable il cubo, in modo che lo sia all'inizio di una nuova simulazione
    sim.setBoolProperty(sim.getObject('/Cuboid'), 'dynamic', True, 0)
    sim.setBoolProperty(sim.getObject('/Cuboid'), 'respondable', True, 0)
    print("Simulazione terminata.")


if __name__ == '__main__':
    main()