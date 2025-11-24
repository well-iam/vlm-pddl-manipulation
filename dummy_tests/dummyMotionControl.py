from tesi_gemini_robotics.implementations.coppelia.components.robot_execution import *
from tesi_gemini_robotics.implementations.coppelia.coppeliasim_setup import *
from tesi_gemini_robotics.implementations.coppelia.components.motion_planning import *

'''
def setup_collections_via_api(sim, robot_name, environment_object_handles):
    """
    Crea e popola le collezioni 'Robot' e 'Environment' dinamicamente via API.
    Questo è il metodo moderno e corretto per le versioni recenti di CoppeliaSim.
    """
    print("Setup dinamico delle collezioni via API...")

    try:
        # --- 1. CREA LE COLLEZIONI (o le trova se esistono già) ---
        robot_collection_handle = sim.createCollection(0)
        print("  - Collezione 'Robot' creata dinamicamente.")

        fake_robot_collection_handle = sim.createCollection(0)

        env_collection_handle = sim.createCollection(0)
        print("  - Collezione 'Environment' creata dinamicamente.")


        # --- 2. POPOLA LA COLLEZIONE 'ROBOT' ---
        # Aggiungiamo l'intero modello del robot (l'albero gerarchico) alla collezione.
        try:
            robot_handle = sim.getObject(f'/{robot_name}')
            # sim.handle_tree dice di includere l'oggetto e tutti i suoi figli.
            # options=0 significa "aggiungi".
            sim.addItemToCollection(robot_collection_handle, sim.handle_tree, robot_handle, 0)
            print(f"  - Aggiunto l'intero albero del robot '{robot_name}' alla collezione 'Robot'.")
            print(f'COLLECTION HANDLES: {sim.getCollectionObjects(robot_collection_handle)}')
        except Exception as e:
            print(f"    - ERRORE: Impossibile trovare o aggiungere il robot '{robot_name}': {e}")

        try:
            fake_robot_handle = sim.getObject('/FakeFranka')
            # sim.handle_tree dice di includere l'oggetto e tutti i suoi figli.
            # options=0 significa "aggiungi".
            sim.addItemToCollection(fake_robot_collection_handle, sim.handle_tree, fake_robot_handle, 0)
            print(f"  - Aggiunto l'intero albero del robot 'Fake{robot_name}' alla collezione 'Robot'.")
            print(f'COLLECTION HANDLES: {sim.getCollectionObjects(fake_robot_collection_handle)}')
        except Exception as e:
            print(f"    - ERRORE: Impossibile trovare o aggiungere il robot '{robot_name}': {e}")

        # --- 3. POPOLA LA COLLEZIONE 'ENVIRONMENT' ---
        for object_handle in environment_object_handles:
            try:
                # sim.handle_single dice di aggiungere solo questo specifico oggetto.
                sim.addItemToCollection(env_collection_handle, sim.handle_single, object_handle, 0)
            except Exception as e:
                print(e)
                print(f"    - Attenzione: Oggetto environment '{name}' non trovato nella scena.")

        print(f'COLLECTION HANDLES: {sim.getCollectionObjects(env_collection_handle)}')
        print("✅ Collezioni create e popolate con successo.")

        return [robot_collection_handle, fake_robot_collection_handle, env_collection_handle]

    except Exception as e:
        print(f"❌ Errore critico durante la gestione delle collezioni: {e}")
        return None

def setup_scene_handles(sim):
    """
    Ottiene e restituisce gli handle per i componenti chiave del robot e della scena
    in modo robusto, cercando gli oggetti per tipo e gerarchia.
    """
    print("Recupero degli handle degli oggetti in modo robusto...")
    handles = {}

    try:
        # 1. OTTENERE GLI OGGETTI DI CONTROLLO PRINCIPALI
        # Questi sono i pochi nomi che è ragionevole mantenere fissi, poiché
        # rappresentano i modelli principali e gli elementi di controllo.
        handles['base'] = sim.getObject('/Franka')
        handles['fake_base'] = sim.getObject('/FakeFranka')
        #handles['gripper'] = sim.getObject('/Franka/FrankaGripper')
        handles['target'] = sim.getObject('/Franka/Franka_target')
        handles['tip'] = sim.getObject('/Franka/Franka_tip')  # Cerca il punto di connessione (end-effector)
        handles['cuboid'] = sim.getObject('/Cuboid')

        # 2. TROVARE I GIUNTI IN MODO DINAMICO (LA PARTE CHIAVE)
        # Invece di cercare 'joint1', 'joint2', ecc., chiediamo al simulatore di darci
        # tutti gli oggetti di tipo "giunto" che sono figli del modello Franka.

        all_robot_joints = sim.getObjectsInTree(handles['base'], sim.object_joint_type, 0)
        print(f"Trovati {len(all_robot_joints)} oggetti di tipo 'joint' in totale sotto '/Franka':")
        for h in all_robot_joints:
            joint_name = sim.getObjectAlias(h) or sim.getObjectName(h)
            # Stampa il nome e l'handle di ogni giunto trovato
            print(f"  - Nome: '{joint_name}', Handle: {h}")

        fake_robot_joints = sim.getObjectsInTree(handles['fake_base'], sim.object_joint_type, 0)

        handles['arm_joints'] = all_robot_joints
        handles['fake_arm_joints'] = fake_robot_joints
        print("\n--- Composizione delle Liste Finali ---")
        print(f"Giunti del BRACCIO ({len(handles['arm_joints'])}): {handles['arm_joints']}")
        print("------------------------------------")

        # 3. OTTENERE LE COLLEZIONI
        # Lista degli oggetti che compongono il tuo ambiente
        all_top_level_shape_handles = sim.getObjectsInTree(sim.handle_scene, sim.sceneobject_shape, 2)
        ignore_list = [handles['base'], handles['fake_base'], sim.getObject('/Floor')]
        env_objects_handles = [h for h in all_top_level_shape_handles if h not in ignore_list]
        #print(f'DEBUG: {all_top_level_shape_handles}')
        #print(f'DEBUG: {env_objects_handles}')

        # Chiama la nuova funzione per creare le collezioni
        collection_handles = setup_collections_via_api(sim, robot_name='Franka', environment_object_handles=env_objects_handles)
        if not collection_handles:
            print("Impossibile creare le collezioni, termino il programma.")
            return

        # Per le collezioni, l'unico modo è usare il loro nome.
        handles['robot_collection'] = collection_handles[0]
        handles['fake_robot_collection'] = collection_handles[1]
        handles['environment_collection'] = collection_handles[2]

        # Riepilogo per verifica
        print(f"  - Trovato robot: '{sim.getObjectAlias(handles['base'])}'")
        print(f"  - Trovati {len(handles['arm_joints'])} giunti del braccio.")
        #print(f"  - Trovati {len(handles['gripper_joints'])} giunti della pinza.")
        print(f"  - Trovato target IK: '{sim.getObjectAlias(handles['target'])}'")

    except Exception as e:
        print(f"❌ ERRORE durante il recupero di un handle: {e}")
        print("    Controlla che tutti gli oggetti principali (es. /Franka, /FrankaGripper, /Franka_target, /Cuboid) "
              "siano presenti e abbiano i nomi corretti nella scena.")
        return None

    print("✅ Handle recuperati con successo.")
    return handles


def execute_path(client, sim, handles, path, tolerance=0.01):
    """Esegue un percorso definito come una lista di configurazioni di giunti."""
    print(f"Esecuzione percorso con {len(path)} waypoint...")

    for i, waypoint in enumerate(path):
        # Invia la configurazione target a tutti i giunti

        for j, joint_handle in enumerate(handles['arm_joints']):
            # print(f'PRIMA ({j}): {sim.getJointTargetPosition(joint_handle)}')
            sim.setJointTargetPosition(joint_handle, waypoint[j])
            # print(f'DOPO ({j}): {sim.getJointTargetPosition(joint_handle)}')

        # Ciclo di controllo sincrono: avanza la simulazione finché il target non è raggiunto
        while True:
            #client.step()  # Avanza di un passo di simulazione

            current_q = [sim.getJointPosition(h) for h in handles['arm_joints']]
            # Calcola la distanza (norma) tra la configurazione attuale e quella target
            dist = np.linalg.norm(np.array(current_q) - np.array(waypoint))
            print(f'dist={dist}') #DEBUG
            if dist < tolerance:
                # print(f"Waypoint {i+1}/{len(path)} raggiunto.")
                break
    print("✅ Percorso completato.")



def setup_ik_environment(sim, simIK, scene_handles):
    """
    Crea l'ambiente IK e TRADUCE gli handle della scena in handle IK.
    """
    print("Setup dell'ambiente di Cinematica Inversa (IK)...")
    try:
        # 1. Crea l'ambiente e il gruppo IK
        ik_env = simIK.createEnvironment()
        ik_group = simIK.createGroup(ik_env)
        #simIK.setGroupCalculation(ik_env, ik_group, simIK.method_pseudo_inverse, 0, 6)

        # 2. Aggiungi la catena cinematica del robot al mondo IK
        #    Questa funzione è la chiave: prende gli handle della SCENA...
        ik_element, sim_to_ik_map, ik_to_sim_map = simIK.addElementFromScene(
            ik_env,
            ik_group,
            scene_handles['base'],
            scene_handles['tip'],
            scene_handles['target'],
            simIK.constraint_pose  # Vincola sia la posizione che l'orientamento
        )

        # 3. TRADUZIONE: Usa la mappa per trovare gli handle IK dei giunti
        #    Prendiamo gli handle dei giunti della scena e troviamo i loro "gemelli" nel mondo IK.
        ik_joint_handles = [sim_to_ik_map[h] for h in scene_handles['arm_joints']]
        # ADDED
        ik_base_handle = sim_to_ik_map[scene_handles['base']]
        ik_tip_handle = sim_to_ik_map[scene_handles['tip']]
        ik_target_handle = sim_to_ik_map[scene_handles['target']]

        # check_scene_joint_handles = [ik_to_sim_map[h] for h in ik_joint_handles]
        # print(check_scene_joint_handles)
        # print(scene_handles)

        # 4. Salva tutto in un dizionario per un uso futuro
        ik_handles = {
            'env': ik_env,
            'group': ik_group,
            'element': ik_element,
            'joints_ik': ik_joint_handles,  # QUESTA è la lista corretta da usare in findConfigs
            'base_ik': ik_base_handle,
            'tip_ik': ik_tip_handle,
            'target_ik': ik_target_handle
        }

        print("✅ Ambiente IK creato e handle tradotti con successo.")
        return ik_handles

    except Exception as e:
        print(f"❌ ERRORE durante la configurazione dell'ambiente IK: {e}")
        return None

def plan_path_rrt(sim, simOMPL, handles, start_config, goal_config):
    """Pianifica un percorso collision-free usando OMPL con l'algoritmo RRTConnect."""
    print("Pianificazione del percorso con RRTConnect...")

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
        for i, h in enumerate(handles['arm_joints']):
            sim.setJointPosition(h, state[i])

        # Esegui il controllo di collisione usando il motore principale di CoppeliaSim
        is_colliding, _ = sim.checkCollision(handles['robot_collection'], handles['environment_collection'])
        if is_colliding:
            print(f'COLLISIONE: {is_colliding}')

        # Ripristina la posa originale del robot
        for i, h in enumerate(handles['arm_joints']):
            sim.setJointPosition(h, original_q[i])

        # Restituisce True se NON c'è collisione (stato valido)
        return not is_colliding
    # Impostazione delle coppie di collisione: il robot non deve collidere con l'ambiente
    simOMPL.setCollisionPairs(task, [handles['robot_collection'], handles['environment_collection']])

    # Impostazione dell'algoritmo
    simOMPL.setAlgorithm(task, simOMPL.Algorithm.RRTConnect) #or RRTstar
    #simOMPL.setStateValidationCallback(task, state_validity_callback)
    # Impostazione stati iniziale e finale
    simOMPL.setStartState(task, start_config)
    simOMPL.setGoalState(task, goal_config)

    # Esecuzione del planning
    simOMPL.setup(task)
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
        print(f"✅ Percorso RRT trovato con {len(path)} waypoint.")
        return path
    else:
        print("❌ Pianificazione RRT fallita. Nessun percorso trovato.")
        return None

def is_state_valid(sim, handles, state_to_check):
    """
    Controlla se una data configurazione di giunti ('state') è valida (non in collisione).
    Questa è la nostra funzione di validazione riutilizzabile.
    """
    original_q = [sim.getJointPosition(h) for h in handles['fake_arm_joints']]
    try:
        sim.setJointPositions(handles['arm_joints'], state_to_check)
        collision_env = sim.checkCollision(handles['robot_collection'], handles['environment_collection'])
        auto_collision = sim.checkCollision(handles['robot_collection'], handles['robot_collection'])
        is_colliding = collision_env or auto_collision
    finally:
        sim.setJointPositions(handles['arm_joints'], original_q)
    return not is_colliding


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
        print(handles)
        #auto_collision = sim.checkCollision(handles['fake_robot_collection'], handles['fake_robot_collection'])

        return not (collision_env) #or auto_collision)
    except Exception as e:
        print(f"  - Errore durante la validazione con il dummy: {e}")
        return False


def find_valid_ik_solution(sim, simIK, ik_handles, handles, target_pose_matrix, max_attempts=10,
                           search_time_per_attempt=1.0):
    """
    Cerca una soluzione IK e la valida immediatamente per le collisioni.
    Riprova a trovare soluzioni alternative se la prima non è valida.
    """
    print("Ricerca di una soluzione IK VALIDA...")

    # Imposta la posa target nel mondo della SCENA (sul target dummy del robot reale)
    simIK.setObjectMatrix(ik_handles['env'], ik_handles['target_ik'], target_pose_matrix, ik_handles['base_ik'])

    for attempt in range(max_attempts):
        print(f"  - Tentativo {attempt + 1}/{max_attempts}...")

        # 1. Esegui la ricerca IK
        #    La funzione può restituire più soluzioni geometriche (es. gomito alto/basso)
        joint_configs = simIK.findConfigs(
            ik_handles['env'],
            ik_handles['group'],
            ik_handles['joints_ik'],
            {'maxTime': search_time_per_attempt}  # Parametro per limitare il tempo di ricerca
        )

        if not joint_configs:
            print("    - L'IK non ha trovato soluzioni geometriche in questo tentativo.")
            continue  # Passa al prossimo tentativo

        print(f"    - Trovate {len(joint_configs)} soluzioni geometriche. Ora le valido...")

        # 2. Itera su TUTTE le soluzioni trovate e testa la prima che è valida
        for i, solution_q in enumerate(joint_configs):
            print(f"      - Validazione soluzione {i + 1}...")

            # 3. Usa il DUMMY robot per il controllo di collisione
            if is_config_valid_with_dummy(sim, handles, solution_q):
                print(f"    - ✅ Trovata una soluzione VALIDA e collision-free al tentativo {attempt + 1}!")
                return solution_q  # Restituisce la prima soluzione valida trovata
            else:
                print(f"      - ❌ Soluzione {i + 1} è in collisione.")

    # Se il loop finisce senza aver restituito una soluzione, significa che tutti i tentativi sono falliti.
    print(f"❌ FALLIMENTO: Nessuna soluzione IK valida e collision-free trovata dopo {max_attempts} tentativi.")
    return None

def solve_ik(sim, simIK, ik_handles, scene_handles, target_pose):
    """
    Risolve la cinematica inversa per una data posa cartesiana del target.
    Restituisce una configurazione di giunti valida o None.
    """
    print("Risoluzione cinematica inversa (IK)...")
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
    #print(f'joint_configs: {joint_configs}') #DEBUG

    if joint_configs:
        print(f"  - Trovate {len(joint_configs)} soluzioni IK. Uso la prima.")
        # La funzione restituisce una lista di soluzioni.
        # Prendiamo la prima, che è la più vicina alla configurazione attuale.
        return joint_configs[0]
    else:
        print("❌ Nessuna soluzione IK trovata per la posa specificata.")
        return None

def solve_linear_path(sim, simIK, ik_handles, scene_handles, target_pose):
    """
    Risolve la cinematica inversa per una data posa cartesiana del target.
    Restituisce una configurazione di giunti valida o None.
    """
    print("Calcolo del percorso cartesiano con simIK.generatePath...")
    simIK.setObjectMatrix(ik_handles['env'], ik_handles['target_ik'], target_pose, ik_handles['base_ik'])
    # Numero di punti desiderati nel percorso risultante
    numero_punti_percorso = 20

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
            print(f"✅ Percorso cartesiano trovato con {len(path_waypoints)} waypoint.")
            return path_waypoints
            # Ora puoi eseguire questo percorso con la tua funzione execute_path
            # execute_path(client, sim, scene_handles, path_waypoints)

        else:
            print("❌ simIK.generatePath non è riuscito a trovare un percorso valido.")
            return None

    except Exception as e:
        print(f"❌ Errore durante l'esecuzione di simIK.generatePath: {e}")

'''

if __name__ == '__main__':

    print('Program started')
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')
    simOMPL = client.require('simOMPL')

    handles = setup_scene_handles(sim)

    #sim.setStepping(True)
    sim.startSimulation()

    # PARTE Q-CONFIG

    # home_q = [0, 0, 0, - np.pi/2, 0, np.pi/2, 0]
    # config_q = [0, 0, 0, - np.pi / 2, 0, np.pi / 2, np.pi/4]
    # execute_path(client, sim, handles, [config_q])
    # time.sleep(3)

    # PARTE IK
    ik_handles = setup_ik_environment(sim, simIK, handles)
    # # TO GIVE WRT BASE (i.e. -0.07(on z-axis) wrt world frame)
    # custom_pose = [1, 0, 0, 0.5955, 0, 1, 0, 0.0, 0, 0, 1, 0.6245 - 0.07]
    # custom_pose = sim.getObjectMatrix(handles['cuboid'],handles['base'])
    # custom_pose[11] += 0.1
    # print(custom_pose)
    # custom_q = solve_ik(sim, simIK, ik_handles, handles, custom_pose)
    # #print(custom_q)
    # execute_path(client, sim, handles, [custom_q])
    # time.sleep(3)

    # PARTE RRT
    # cuboid_pose = sim.getObjectMatrix(handles['cuboid'], handles['base'])
    # pre_grasp_pose = cuboid_pose
    # pre_grasp_pose[11] += 0.1  # z_index=11
    dummy = sim.getObject('/dummy')
    dummy_pose = sim.getObjectMatrix(dummy,handles['base'])

    current_q = [sim.getJointPosition(h) for h in handles['arm_joints']]
    #pre_grasp_q = solve_ik(sim, simIK, ik_handles, handles, dummy_pose)

    #sim.setObjectMatrix(handles['base'],[1,0,0,0,0,1,0,0,0,0,1,0.5])

    # for i, h in enumerate(handles['fake_arm_joints']):
    #     sim.setJointPosition(h, pre_grasp_q[i])
    #
    # res, collidingObjectHandles = sim.checkCollision(handles['fake_robot_collection'], handles['environment_collection'])
    # print(f'RES: {res}')
    # print(f'COLLISION BETWEEN FAKE ROBOT AND ENV: {collidingObjectHandles}')
    # time.sleep(5)
    #
    # for i, h in enumerate(handles['fake_arm_joints']):
    #     sim.setJointPosition(h, current_q[i])
    # time.sleep(5)

    # Chiama il nuovo risolutore "intelligente"
    # pre_grasp_q = find_valid_ik_solution(sim, simIK, ik_handles, handles, dummy_pose)
    # if pre_grasp_q:
    #     path_to_pre_grasp = plan_path_rrt(sim, simOMPL, handles, current_q, pre_grasp_q)
    #     if path_to_pre_grasp:
    #         execute_path(client, sim, handles, path_to_pre_grasp)
    #     else:
    #         print("FALLITO")

    # Percorso lineare
    path_waypoints = solve_linear_path(simIK, ik_handles, dummy_pose)
    execute_path(client, sim, handles, path_waypoints)

    input("Premi qualcosa per terminare")
    sim.stopSimulation()
    print('Program ended')