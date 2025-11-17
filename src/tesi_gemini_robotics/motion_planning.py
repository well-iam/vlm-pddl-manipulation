import logging
import time
import numpy as np

logger = logging.getLogger(__name__)

def is_config_valid_with_dummy(sim, handles, config_to_check):
    """
    Usa il robot DUMMY per verificare se una data configurazione è valida.
    Questa funzione è sicura perché opera su un modello non dinamico.
    """
    try:
        # 1. Applica la configurazione al DUMMY robot
        for i, h in enumerate(handles['fake_arm_joints']):
            sim.setJointPosition(h, config_to_check[i])
        time.sleep(3)

        # 2. Controlla le collisioni del DUMMY
        collision_env, handles = sim.checkCollision(handles['fake_robot_collection'], handles['environment_collection'])
        # obj1_name = sim.getObjectAlias(handles[0]) or sim.getObjectName(handles[0])
        # obj2_name = sim.getObjectAlias(handles[1]) or sim.getObjectName(handles[1])
        #auto_collision = sim.checkCollision(handles['fake_robot_collection'], handles['fake_robot_collection'])
        return handles, not (collision_env) #or auto_collision)

    except Exception as e:
        logger.exception(f"  - Errore durante la validazione con il dummy: ")
        return False

def solve_ik(simIK, ik_handles, target_pose):
    """
    Risolve la cinematica inversa per una data posa cartesiana del target.
    Restituisce una configurazione di giunti valida o None.
    """
    logger.debug("Risoluzione cinematica inversa (IK)...")
    current_pos = simIK.getObjectMatrix(ik_handles['env'], ik_handles['target_ik'], ik_handles['base_ik'])
    #print(f'CURRENT_POS: {current_pos}')

    simIK.setObjectMatrix(ik_handles['env'], ik_handles['target_ik'], target_pose, ik_handles['base_ik'])
    # changed_pos = simIK.getObjectMatrix(ik_handles['env'], ik_handles['target_ik'], ik_handles['base_ik'])
    #print(f'CHANGED_POS: {changed_pos}')

    # Esegui la ricerca IK
    # Ora esegui la ricerca usando gli handle del MONDO IK
    # res, reason, _ = simIK.handleGroup(ik_handles['env'], ik_handles['group'])
    # if res!= simIK.result_success:
    #     print(f'Ragione: {reason}')
    #     return None
    joint_configs = simIK.findConfigs(
        ik_handles['env'],
        ik_handles['group'],
        ik_handles['joints_ik'],  # <-- Passiamo la lista di handle IK tradotti!
        {}
    )
    logger.debug(f'joint_configs: {joint_configs}') #DEBUG

    if joint_configs:
        logger.debug(f"  - Trovate {len(joint_configs)} soluzioni IK. Uso la prima.")
        # La funzione restituisce una lista di soluzioni.
        # Prendiamo la prima, che è la più vicina alla configurazione attuale.
        return joint_configs[0]
    else:
        logger.error("❌ Nessuna soluzione IK trovata per la posa specificata.")
        return None

def find_valid_ik_solution(sim, simIK, ik_handles, handles, target_pose_matrix, max_attempts=10,
                           search_time_per_attempt=1.0):
    """
    Cerca una soluzione IK e la valida immediatamente per le collisioni.
    Riprova a trovare soluzioni alternative se la prima non è valida.
    """
    logger.debug("Ricerca di una soluzione IK VALIDA...")

    # Imposta la posa target nel mondo della SCENA (sul target dummy del robot reale)
    simIK.setObjectMatrix(ik_handles['env'], ik_handles['target_ik'], target_pose_matrix, ik_handles['base_ik'])

    for attempt in range(max_attempts):
        logger.debug(f"  - Tentativo {attempt + 1}/{max_attempts}...")

        # 1. Esegui la ricerca IK
        #    La funzione può restituire più soluzioni geometriche (es. gomito alto/basso)
        joint_configs = simIK.findConfigs(
            ik_handles['env'],
            ik_handles['group'],
            ik_handles['joints_ik'],
            {'maxTime': search_time_per_attempt}  # Parametro per limitare il tempo di ricerca
        )

        if not joint_configs:
            logger.debug("    - L'IK non ha trovato soluzioni geometriche in questo tentativo.")
            continue  # Passa al prossimo tentativo

        logger.debug(f"    - Trovate {len(joint_configs)} soluzioni geometriche. Ora le valido...")
        # 2. Itera su TUTTE le soluzioni trovate e testa la prima che è valida
        for i, solution_q in enumerate(joint_configs):
            logger.debug(f"      - Validazione soluzione {i + 1}...")
            # 3. Usa il DUMMY robot per il controllo di collisione
            collision_handles, config_valid = is_config_valid_with_dummy(sim, handles, solution_q)
            if config_valid:
                logger.debug(f"    - ✅ Trovata una soluzione VALIDA e collision-free al tentativo {attempt + 1}!")
                return solution_q  # Restituisce la prima soluzione valida trovata
            else:
                logger.warning(f"      - ❌ Collisione tra {collision_handles}.")

    # Se il loop finisce senza aver restituito una soluzione, significa che tutti i tentativi sono falliti.
    logger.error(f"❌ FALLIMENTO: Nessuna soluzione IK valida e collision-free trovata dopo {max_attempts} tentativi.")
    return None

def solve_linear_path(simIK, ik_handles, target_pose):
    """
    Risolve la cinematica inversa per una data posa cartesiana del target.
    Restituisce una configurazione di giunti valida o None.
    """
    logger.debug("Calcolo del percorso cartesiano con simIK.generatePath...")
    simIK.syncFromSim(ik_handles['env'], [ik_handles['group']])
    simIK.setObjectMatrix(ik_handles['env'], ik_handles['target_ik'], target_pose, ik_handles['base_ik'])
    # Numero di punti desiderati nel percorso risultante
    numero_punti_percorso = 10

    try:
        # Chiama la funzione passando gli handle IK corretti e la callback
        config_list_flat = simIK.generatePath(
            ik_handles['env'],  # Handle dell'ambiente IK
            ik_handles['group'],  # Handle del gruppo IK
            ik_handles['joints_ik'],  # Handle IK dei giunti (per l'output)
            ik_handles['tip_ik'],  # Handle SCENA (???) della punta (richiesto dalla funzione)
            numero_punti_percorso,  # Quanti punti generare
            #lambda state: is_state_valid(sim, scene_handles, state)  # La nostra callback
        )

        if config_list_flat:
            # Riorganizza l'output piatto in una lista di waypoint
            num_joints = len(ik_handles['joints_ik'])
            num_points = len(config_list_flat) // num_joints
            path_waypoints = [config_list_flat[i * num_joints: (i + 1) * num_joints] for i in range(num_points)]
            logger.debug(f"✅ Percorso cartesiano trovato con {len(path_waypoints)} waypoint.")
            return path_waypoints

        else:
            logger.error("❌ simIK.generatePath non è riuscito a trovare un percorso valido.")
            return None

    except Exception as e:
        logger.exception(f"❌ Errore durante l'esecuzione di simIK.generatePath: ")

def plan_path_rrt(sim, simOMPL, handles, goal_config):
    """Pianifica un percorso collision-free usando OMPL con l'algoritmo RRTConnect."""
    logger.debug("Pianificazione del percorso con RRTConnect...")
    start_config = [sim.getJointPosition(h) for h in handles['arm_joints']]
    # Definizione dello spazio degli stati (i 7 giunti del braccio)
    #state_space = simOMPL.createStateSpace('j', simOMPL.StateSpaceType.real, handles['arm_joints'], [-3.14, 3.14])

    # Creazione del task di planning
    task = simOMPL.createTask('task')
    #simOMPL.setStateSpace(task, state_space)
    # 1. CREAZIONE E ASSEGNAZIONE DELLO SPAZIO DEGLI STATI IN UN COLPO SOLO
    # Questa singola riga sostituisce l'intero blocco for precedente.
    # - task: a quale problema stiamo assegnando lo spazio.
    # - handles['arm_joints']: la lista dei giunti da cui creare lo spazio.
    #simOMPL.setStateSpaceForJoints(task, handles['arm_joints'])

    # 1. DEFINIZIONE DELLA PROIEZIONE
    # Creiamo una lista di flag. Mettiamo a 1 i primi due giunti,
    # dicendo a OMPL di usare questi per la sua "mappa 2D".
    # Devi dirgli esplicitamente quali giunti usare per creare la "mappa 2D".
    # Solitamente si scelgono i primi 2 o 3 giunti, perché sono quelli che determinano i movimenti più ampi del braccio.
    num_joints = len(handles['arm_joints'])
    projection_setup = [1, 1, 1, 0, 0, 0, 0]  # Esempio: [1, 1, 0, 0, 0, 0, 0]

    # 2. CREAZIONE DELLO SPAZIO DEGLI STATI CON LA PROIEZIONE
    # Ora passiamo la lista 'projection_setup' come terzo argomento.
    simOMPL.setStateSpaceForJoints(task, handles['arm_joints'], projection_setup)

    # Questa funzione è il cuore del controllo collisioni.
    def state_validity_callback(state):
        # Salva la posa attuale del robot per poterla ripristinare
        original_q = [sim.getJointPosition(h) for h in handles['arm_joints']]

        # Imposta temporaneamente il robot nella configurazione 'state' da testare
        for i, h in enumerate(handles['fake_arm_joints']):
            sim.setJointPosition(h, state[i])

        # Esegui il controllo di collisione usando il motore principale di CoppeliaSim
        is_colliding_RE, _ = sim.checkCollision(handles['fake_robot_collection'], handles['environment_collection'])
        if is_colliding_RE:
            print(f'COLLISIONE Robot-Ambiente: {is_colliding_RE}')

        is_colliding_RR, _ = sim.checkCollision(handles['fake_robot_collection'], handles['fake_robot_collection'])
        if is_colliding_RR:
            print(f'COLLISIONE Robot-Robot: {is_colliding_RR}')

        # Ripristina la posa originale del robot
        for i, h in enumerate(handles['fake_arm_joints']):
            sim.setJointPosition(h, original_q[i])

        # Restituisce True se NON c'è collisione (stato valido)
        return not (is_colliding_RE or is_colliding_RR)
    # Impostazione delle coppie di collisione: il robot non deve collidere con l'ambiente
    simOMPL.setCollisionPairs(task, [
        handles['fake_robot_collection'], handles['environment_collection'],
        handles['fake_robot_collection'], handles['fake_robot_collection']])

    # Impostazione dell'algoritmo
    simOMPL.setAlgorithm(task, simOMPL.Algorithm.RRTConnect) #or RRTstar
    # simOMPL.setStateValidationCallback(task, state_validity_callback)

    # Impostazione stati iniziale e finale
    simOMPL.setStartState(task, start_config)
    simOMPL.setGoalState(task, goal_config)

    # Setup task
    simOMPL.setup(task)

    # Sanifica stato iniziale
    logger.debug(f"Stato iniziale (raw): {np.round(start_config, 4)}")
    valid_start_config = simOMPL.enforceBounds(task, start_config)
    logger.debug(f"Stato iniziale (enforced): {np.round(valid_start_config, 4)}")
    simOMPL.setStartState(task, valid_start_config)

    #Esecuzione del planning
    solved, path_raw = simOMPL.compute(task, 5.0)  # 5 secondi di tempo massimo
    # print(f'SOLVED: {solved}')
    # print(f'LENGTH PATH: {len(path)}')

    # Pulizia
    simOMPL.destroyTask(task)

    if solved:
        # L'output è una lista piatta [p1_j1, p1_j2..., p2_j1, p2_j2...].
        # Dobbiamo raggrupparla in una lista di liste (waypoint).
        num_joints = len(handles['arm_joints'])
        num_points = len(path_raw) // num_joints
        path = [path_raw[i * num_joints: (i + 1) * num_joints] for i in range(num_points)]
        logger.debug(f"✅ Percorso RRT trovato con {len(path)} waypoint.")
        return path
    else:
        logger.error("❌ Pianificazione RRT fallita. Nessun percorso trovato.")
        return None