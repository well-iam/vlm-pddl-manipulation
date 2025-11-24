import logging
import time
import numpy as np

logger = logging.getLogger(__name__)

class CoppeliaPlanner:
    def __init__(self, sim, simIK, simOMPL, scene_handles):
        self.sim = sim
        self.simIK = simIK
        self.simOMPL = simOMPL
        self.scene_handles = scene_handles

        self.ik_handles = self._setup_ik_environment()
        if not self.ik_handles:
            raise RuntimeError("Impossibile inizializzare il sistema di movimento (IK Setup fallito).")

    def _setup_ik_environment(self):
        """
        Crea l'ambiente IK e TRADUCE gli handle della scena in handle IK.
        """
        logger.info("Setup dell'ambiente di Cinematica Inversa (IK)...")
        simIK = self.simIK
        scene_handles = self.scene_handles

        try:
            # 1. Crea l'ambiente e il gruppo IK
            ik_env = simIK.createEnvironment()
            ik_group = simIK.createGroup(ik_env)
            # simIK.setGroupCalculation(ik_env, ik_group, simIK.method_pseudo_inverse, 0, 6)

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
            # print(ik_element)
            # print(sim_to_ik_map)
            # print(ik_to_sim_map)
            # print(scene_handles['arm_joints'])
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

            logger.info("✅ Ambiente IK creato e handle tradotti con successo.")
            return ik_handles

        except Exception as e:
            logger.exception(f"❌ ERRORE durante la configurazione dell'ambiente IK: ")
            return None

    def _set_dummy_config(self, config_to_check):
        for i, h in enumerate(self.scene_handles['fake_arm_joints']):
            self.sim.setJointPosition(h, config_to_check[i])

    def _get_dummy_config(self):
        return [self.sim.getJointPosition(h) for h in self.scene_handles['fake_arm_joints']]

    def _enforce_limits(self, config, safety_margin=0.001):
        """
        Sanifica una configurazione di giunti assicurandosi che rispetti i limiti
        imposti in CoppeliaSim. Sostituisce simOMPL.enforceBounds.
        """
        sim = self.sim
        clamped_config = list(config)
        joints = self.scene_handles['arm_joints']

        for i, joint_handle in enumerate(joints):
            # Ottieni l'intervallo del giunto [min, range]
            # Nota: getJointInterval restituisce (bool, interval)
            is_cyclic, interval = sim.getJointInterval(joint_handle)

            # Verifica se il giunto è ciclico (es. rotazione continua)
            # I giunti ciclici non hanno limiti rigidi, quindi li saltiamo
            if not is_cyclic:
                min_val = interval[0]
                max_val = min_val + interval[1]  # interval[1] è l'ampiezza (range)

                # Applica il margine di sicurezza
                safe_min = min_val + safety_margin
                safe_max = max_val - safety_margin

                # Clamping
                if clamped_config[i] < safe_min:
                    clamped_config[i] = safe_min
                elif clamped_config[i] > safe_max:
                    clamped_config[i] = safe_max

        return clamped_config

    def _is_config_valid_with_dummy(self, config_to_check):
        """
        Usa il robot DUMMY per verificare se una data configurazione è valida.
        Questa funzione è sicura perché opera su un modello non dinamico.
        """
        sim = self.sim
        scene_handles = self.scene_handles
        try:
            # 1. Applica la configurazione al DUMMY robot
            current_config = self._get_dummy_config()
            self._set_dummy_config(config_to_check)
            time.sleep(3)

            # 2. Controlla le collisioni del DUMMY
            collision_env, collision_handles = sim.checkCollision(scene_handles['fake_robot_collection'], scene_handles['environment_collection'])
            self._set_dummy_config(current_config)
            # obj1_name = sim.getObjectAlias(handles[0]) or sim.getObjectName(handles[0])
            # obj2_name = sim.getObjectAlias(handles[1]) or sim.getObjectName(handles[1])
            #auto_collision = sim.checkCollision(handles['fake_robot_collection'], handles['fake_robot_collection'])
            return collision_handles, not (collision_env) #or auto_collision)

        except Exception as e:
            logger.exception(f"  - Errore durante la validazione con il dummy: ")
            return False

    # TODO
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

    def find_valid_ik_solution(self, target_pose_matrix, max_attempts=10, search_time_per_attempt=1.0):
        """
        Cerca una soluzione IK e la valida immediatamente per le collisioni.
        Riprova a trovare soluzioni alternative se la prima non è valida.
        """
        logger.debug("Ricerca di una soluzione IK VALIDA...")
        simIK = self.simIK
        ik_handles = self.ik_handles

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
                logger.debug(""""
                    - L'IK non ha trovato soluzioni geometriche in questo tentativo.
                    A reason for a non-successful operation can be: 
                     - there are some forbidden poses/configurations on the way
                     - some of the configuration points cannot be reached (e.g. out of reach, or due to joint limits).""")
                continue  # Passa al prossimo tentativo

            logger.debug(f"    - Trovate {len(joint_configs)} soluzioni geometriche. Ora le valido...")
            # 2. Itera su TUTTE le soluzioni trovate e testa la prima che è valida
            for i, solution_q in enumerate(joint_configs):
                logger.debug(f"      - Validazione soluzione {i + 1}...")
                # 3. Usa il DUMMY robot per il controllo di collisione
                collision_handles, config_valid = self._is_config_valid_with_dummy(solution_q)
                if config_valid:
                    logger.debug(f"    - ✅ Trovata una soluzione VALIDA e collision-free al tentativo {attempt + 1}!")
                    return solution_q  # Restituisce la prima soluzione valida trovata
                else:
                    logger.warning(f"      - ❌ Collisione tra {collision_handles}.")

        # Se il loop finisce senza aver restituito una soluzione, significa che tutti i tentativi sono falliti.
        logger.error(f"❌ FALLIMENTO: Nessuna soluzione IK valida e collision-free trovata dopo {max_attempts} tentativi.")
        return None

    def plan_linear_path(self, target_pose):
        """
        Risolve la cinematica inversa per una data posa cartesiana del target.
        Restituisce una configurazione di giunti valida o None.
        """
        logger.debug("Calcolo del percorso cartesiano con simIK.generatePath...")
        simIK = self.simIK
        ik_handles = self.ik_handles

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
                self._set_dummy_config(path_waypoints[-1])
                return path_waypoints

            else:
                logger.error("❌ simIK.generatePath non è riuscito a trovare un percorso valido.")
                return None

        except Exception as e:
            logger.exception(f"❌ Errore durante l'esecuzione di simIK.generatePath: ")

    def plan_path_rrt(self, goal_config):
        """Pianifica un percorso collision-free usando OMPL con l'algoritmo RRTConnect."""
        logger.debug("Pianificazione del percorso con RRTConnect...")
        sim = self.sim
        simOMPL = self.simOMPL
        handles = self.scene_handles

        start_config = [sim.getJointPosition(h) for h in handles['arm_joints']]
        # Definizione dello spazio degli stati (i 7 giunti del braccio)
        #state_space = simOMPL.createStateSpace('j', simOMPL.StateSpaceType.real, handles['arm_joints'], [-3.14, 3.14])

        # Creazione del task di planning
        task = simOMPL.createTask('task')

        # Impostazione dell'algoritmo
        simOMPL.setAlgorithm(task, simOMPL.Algorithm.BiTRRT)  # or RRTConnect, or RRTstar

        # CREAZIONE E ASSEGNAZIONE DELLO SPAZIO DEGLI STATI
        # DEFINIZIONE DELLA PROIEZIONE
        # Creiamo una lista di flag. Mettiamo a 1 i primi due giunti,
        # dicendo a OMPL di usare questi per la sua "mappa 2D".
        # Solitamente si scelgono i primi 2 o 3 giunti, perché sono quelli che determinano i movimenti più ampi del braccio.
        projection_setup = [1, 1, 0, 0, 0, 0, 0]  # Esempio: [1, 1, 0, 0, 0, 0, 0]
        # CREAZIONE DELLO SPAZIO DEGLI STATI CON LA PROIEZIONE
        simOMPL.setStateSpaceForJoints(task, handles['arm_joints'], projection_setup)

        # Questa funzione è il cuore del controllo collisioni.
        def state_validity_callback(state):
            # Salva la posa attuale del robot per poterla ripristinare
            # original_q = [sim.getJointPosition(h) for h in handles['arm_joints']]

            # Imposta temporaneamente il robot nella configurazione 'state' da testare
            self._set_dummy_config(state)

            # Esegui il controllo di collisione usando il motore principale di CoppeliaSim
            is_colliding_RE, _ = sim.checkCollision(handles['fake_robot_collection'], handles['environment_collection'])
            if is_colliding_RE:
                print(f'COLLISIONE Robot-Ambiente: {is_colliding_RE}')

            is_colliding_RR, _ = sim.checkCollision(handles['fake_robot_collection'], handles['fake_robot_collection'])
            if is_colliding_RR:
                print(f'COLLISIONE Robot-Robot: {is_colliding_RR}')

            # Ripristina la posa originale del robot
            # self._set_dummy_config(original_q)

            # Restituisce True se NON c'è collisione (stato valido)
            return not (is_colliding_RE or is_colliding_RR)

        # Impostazione delle coppie di collisione: il robot non deve collidere con l'ambiente
        # simOMPL.setCollisionPairs(task, [
        #     handles['robot_collection'], handles['environment_collection']])
        #     # handles['robot_collection'], handles['robot_collection']])    # Dà "invalid state", forse devi usare la custom callback

        simOMPL.setStateValidationCallback(task, state_validity_callback)

        # Impostazione stati iniziale e finale
        # Sanifica stato iniziale
        logger.debug(f"Stato iniziale (raw): {np.round(start_config, 4)}")
        valid_start_config = self._enforce_limits(start_config)
        logger.debug(f"Stato iniziale (enforced): {np.round(valid_start_config, 4)}")

        simOMPL.setStartState(task, valid_start_config)
        simOMPL.setGoalState(task, goal_config)

        # Setup task
        simOMPL.setup(task)

        #Esecuzione del planning
        solved, path_raw = simOMPL.compute(task, 5.0)  # 5 secondi di tempo massimo
        # print(f'SOLVED: {solved}')
        # print(f'LENGTH PATH: {len(path)}')

        # Pulizia
        simOMPL.destroyTask(task)

        if solved:
            self._set_dummy_config(goal_config)
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