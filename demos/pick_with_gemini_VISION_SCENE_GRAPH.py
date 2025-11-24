from tesi_gemini_robotics.implementations.coppelia.coppeliasim_setup import connect_to_sim
from tesi_gemini_robotics.implementations.coppelia.robot_controller import RobotController
from tesi_gemini_robotics.core.llm_interface import GeminiClient
from tesi_gemini_robotics.utils import get_image_of_scene, get_top_level_interactive_objects, get_current_scene_graph_json
import time

SYSTEM_INSTRUCTION_TEMPLATE = """
Sei un pianificatore robotico di alto livello. Il tuo unico compito è analizzare lo stato del mondo e l'obiettivo dell'utente, e decidere quale skill, da una lista predefinita, deve essere eseguita.

**REGOLE FONDAMENTALI:**
1.  Rispondi SEMPRE E SOLO con un singolo oggetto JSON.
2.  Non includere mai testo al di fuori dell'oggetto JSON (nemmeno "Certo, ecco il JSON:").
3.  Basa la tua decisione esclusivamente sulla "Descrizione Scena" fornita in questo contesto e sull'obiettivo dell'utente.
4.  La tua risposta JSON deve contenere le seguenti chiavi: "ragionamento", "skill_scelta", "argomenti".

**SKILL DISPONIBILI:**
Le uniche skill che puoi scegliere sono:
* `pick_and_hold`: Scegli questa skill se devi afferrare un oggetto. Richiede un argomento: ["nome_oggetto"].
* `pick_and_place`: Scegli questa skill se devi prendere un oggetto e posizionarlo in un luogo specifico. Richiede due argomenti: ["nome_oggetto", "nome_luogo"].
* `done`: Scegli questa skill se l'obiettivo è già completato o se è impossibile da eseguire. Richiede zero argomenti: [].

**DESCRIZIONE SCENA (STATICA):**
Questo è lo stato attuale del mondo, che non cambierà per questa sessione:
{scene_graph}

**LOGICA DECISIONALE OBBLIGATORIA:**
* Se l'utente ti chiede di afferrare un oggetto, controlla la sua proprietà "raggiungibile" nello Scene Graph.
* Se "raggiungibile" è `true`, devi scegliere la skill `pick_and_hold` con quell'oggetto come argomento.
* Se "raggiungibile" è `false`, devi scegliere la skill `done` e devi spiegare nel campo "ragionamento" che l'oggetto è fuori portata.

**ESEMPIO DI OUTPUT PER UN OGGETTO RAGGIUNGIBILE:**
{{
  "ragionamento": "L'utente vuole il Cuboid. Dallo Scene Graph vedo che 'raggiungibile' è true, quindi procedo con l'azione di presa.",
  "skill_scelta": "pick_and_hold",
  "argomenti": ["Cuboid"]
}}

**ESEMPIO DI OUTPUT PER UN OGGETTO NON RAGGIUNGIBILE:**
{{
  "ragionamento": "L'utente vuole la SferaVerde. Dallo Scene Graph vedo che 'raggiungibile' è false. È impossibile completare l'azione.",
  "skill_scelta": "done",
  "argomenti": []
}}
"""

def main():
    # INIZIALIZZAZIONE SIMULAZIONE
    client, sim, simIK, simOMPL = connect_to_sim()
    if not client: return

    # INIZIALIZZAZIONE DI CLASSI ROBOT E GEMINI_CLIENT
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
        print(e)
        return  # Esce se non può creare il client




    # --- Inizio Simulazione ---
    sim.startSimulation()
    time.sleep(0.2) #Serve per dare tempo ai buffer del sensore di immagini di riempirsi

    # Ottieni la lista dei nomi degli oggetti dal simulatore
    objects_to_ignore = ['Floor', 'Franka', 'FakeFranka']
    available_objects = get_top_level_interactive_objects(sim, ignore_list=objects_to_ignore)

    # Acquisizione immagine della scena
    immagine_scena = get_image_of_scene(sim)

    # Gemini identifica gli oggetti con i loro nomi in simulazione
    identified_objects = gemini_planner.get_object_list_from_image(immagine_scena, available_objects)
    if not identified_objects:
        print("Nessun oggetto identificato. Termino il programma.")
        sim.stopSimulation()
        return

    # Costruzione del Scene Graph
    scene_json_string = get_current_scene_graph_json(sim, identified_objects)

    # Formattazione SYSTEM INSTRUCTION
    SYSTEM_INSTRUCTION = SYSTEM_INSTRUCTION_TEMPLATE.format(
        scene_graph=scene_json_string
    )
    gemini_planner.update_system_instruction(SYSTEM_INSTRUCTION)

    # Richiesta utente
    obiettivo_utente = "Afferra il cubo."

    # # Usa il metodo della classe per ottenere la scelta
    # scelta_gemini = gemini_planner.get_next_skill_from_image(
    #     immagine_scena,
    #     obiettivo_utente,
    #     list(robot.available_skills.keys()),
    #     identified_objects
    # )

    # 4. Esegui le query successive usando l'istruzione dinamica
    print("\n--- Fase 2: Ciclo di Azioni Guidato ---")
    primo_comando_utente = "Afferra il cubo."

    # Per tutte le chiamate successive, passi la stessa istruzione di sistema.
    scelta_gemini = gemini_planner.ask_for_skill_json(primo_comando_utente, list(robot.available_skills.keys()))

    # print(f"\nRisposta del modello:\n{scelta_gemini}")

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

    input_utente = input('Secondo prompt per modello: ')
    gemini_planner.send_chat_message(input_utente)

    print(gemini_planner.chat_history)

    # --- Fine Simulazione ---
    input('Premi INVIO per terminare la simulazione.')
    sim.stopSimulation()

    # (TEMPORANEO) Setto dynamic e respondable il cubo, in modo che lo sia all'inizio di una nuova simulazione
    sim.setBoolProperty(sim.getObject('/Cuboid'), 'dynamic', True, 0)
    sim.setBoolProperty(sim.getObject('/Cuboid'), 'respondable', True, 0)
    print("Simulazione terminata.")


if __name__ == '__main__':
    main()