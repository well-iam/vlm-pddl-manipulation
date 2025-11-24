import logging
from tesi_gemini_robotics.interfaces.robot_interface import RobotInterface
from .components.perception import CoppeliaPerception
from .components.planning import CoppeliaPlanner
from .components.actuation import CoppeliaActuator
from .components.gripper import CoppeliaGripper

logger = logging.getLogger(__name__)

class CoppeliaSimRobot(RobotInterface):
    """
    Classe che incapsula la connessione e il controllo di un robot in CoppeliaSim.

    Gestisce gli handle, l'ambiente IK/OMPL e fornisce metodi di alto livello
    per eseguire le skill del robot (es. pick_and_hold).
    """
    def __init__(self, client, sim, simIK, simOMPL, robot_name='Franka'):
        """
        Inizializza il controller del robot.

        Args:
            client: Oggetto client ZMQ principale.
            sim: Oggetto client API 'sim'.
            simIK: Oggetto client API 'simIK'.
            simOMPL: Oggetto client API 'simOMPL'.
            robot_name (str): Nome del robot nella scena CoppeliaSim.

        Raises:
            Exception: Se il setup iniziale fallisce (es. handle non trovati).
        """

        logger.info("--- Inizializzazione RobotController ---")
        self.client, self.sim, self.simIK, self.simOMPL = client, sim, simIK, simOMPL
        self.robot_name = robot_name

        # 1. Recupera Handles della Scena
        self.handles = self._setup_scene_handles()
        if not self.handles:
            raise RuntimeError("Setup fallito: impossibile recuperare gli handle della scena.")

        self.perception = CoppeliaPerception(self.sim, self.handles)
        self.planner = CoppeliaPlanner(self.sim, self.simIK, self.simOMPL, self.handles)
        self.actuator = CoppeliaActuator(self.client, self.sim, self.handles)
        self.gripper = CoppeliaGripper(self.sim, self.handles)


        # Mappa Nomi Skill -> Funzioni Python
        # Questo dizionario collega il nome che Gemini sceglierà alla funzione da eseguire.
        self.available_skills = {
            "pick_and_hold": self.perform_pick_and_hold,
            "place": self.perform_place,
            "done": self.perform_done
        }
        self.available_detectors = {
            "pick_and_hold": self.pick_and_hold_success_detector,
            "place": self.place_success_detector,
        }
        logger.debug(f"✅ Skill disponibili per il robot: {list(self.available_skills.keys())}")
        logger.info("✅ RobotController inizializzato con successo.")

    def _setup_scene_handles(self):
        """
        Ottiene e restituisce gli handle per i componenti chiave del robot e della scena
        in modo robusto, cercando gli oggetti per tipo e gerarchia.
        """
        sim = self.sim
        logger.debug("Recupero degli handle degli oggetti nella scena...")
        handles = {}

        try:
            # 1. OTTENERE GLI OGGETTI DI CONTROLLO PRINCIPALI
            # Questi sono i pochi nomi che è ragionevole mantenere fissi, poiché
            # rappresentano i modelli principali e gli elementi di controllo.
            handles['base'] = sim.getObject('/Franka')
            handles['fake_base'] = sim.getObject('/FakeFranka')
            handles['gripper'] = sim.getObject('/Franka/FrankaGripper')
            handles['target'] = sim.getObject('/Franka/Franka_target')
            handles['tip'] = sim.getObject('/Franka/Franka_tip')  # Cerca il punto di connessione (end-effector)

            # 2. TROVARE I GIUNTI IN MODO DINAMICO (LA PARTE CHIAVE)
            # Invece di cercare 'joint1', 'joint2', ecc., chiediamo al simulatore di darci
            # tutti gli oggetti di tipo "giunto" che sono figli del modello Franka.
            all_robot_joints = sim.getObjectsInTree(handles['base'], sim.object_joint_type, 0)

            # Giunti del braccio robotico
            joint_names = [
                f"  - Nome: '{sim.getObjectAlias(h)}', Handle: {h}"
                for h in all_robot_joints
            ]
            joint_list_string = "\n".join(joint_names)
            logger.debug(f"""
            Trovati {len(all_robot_joints)} oggetti di tipo 'joint' in totale sotto '/Franka':
            {joint_list_string}""")

            # Giunti del gripper
            gripper_joints_handles = sim.getObjectsInTree(handles['gripper'], sim.object_joint_type, 0)
            arm_joints_handles = [h for h in all_robot_joints if h not in gripper_joints_handles]

            # Giunti del braccio robotico fake
            fake_robot_joints = sim.getObjectsInTree(handles['fake_base'], sim.object_joint_type, 0)

            handles['arm_joints'] = arm_joints_handles
            handles['gripper_joints'] = gripper_joints_handles
            handles['fake_arm_joints'] = fake_robot_joints
            logger.debug(f"""
            --- Composizione delle Liste Finali ---
              Giunti del BRACCIO ({len(handles['arm_joints'])}): {handles['arm_joints']}
            ------------------------------------
            """)

            # 3. OTTENERE LE COLLEZIONI
            # Lista degli oggetti che compongono il tuo ambiente
            all_top_level_shape_handles = sim.getObjectsInTree(sim.handle_scene, sim.sceneobject_shape, 2)
            ignore_list = [handles['base'], handles['fake_base']]#, sim.getObject('/Floor')]
            env_objects_handles = [h for h in all_top_level_shape_handles if h not in ignore_list]
            logger.debug(all_top_level_shape_handles)
            logger.debug(env_objects_handles)

            # Chiama la nuova funzione per creare le collezioni
            collection_handles = self._setup_collections_via_api(env_objects_handles)
            if not collection_handles:
                logger.critical("Impossibile creare le collezioni, termino il programma.")
                return None

            # Per le collezioni, l'unico modo è usare il loro nome.
            handles['robot_collection'] = collection_handles[0]
            handles['fake_robot_collection'] = collection_handles[1]
            handles['environment_collection'] = collection_handles[2]

            # Riepilogo per verifica
            logger.debug(f"  - Trovato robot: '{sim.getObjectAlias(handles['base'])}'")
            logger.debug(f"  - Trovati {len(handles['arm_joints'])} giunti del braccio.")
            logger.debug(f"  - Trovati {len(handles['gripper_joints'])} giunti della pinza.")
            logger.debug(f"  - Trovato target IK: '{sim.getObjectAlias(handles['target'])}'")

        except Exception as e:
            logger.exception(f"❌ ERRORE durante il recupero di un handle: ")
            logger.error(
                "    Controlla che tutti gli oggetti principali (es. /Franka, /FrankaGripper, /Franka_target, /Cuboid) "
                "siano presenti e abbiano i nomi corretti nella scena.")
            return None

        logger.info("✅ Handle recuperati con successo.")
        return handles

    #TODO
    def _setup_collections_via_api(self, environment_object_handles):
        """
        Crea e popola le collezioni 'Robot' e 'Environment' dinamicamente via API.
        Questo è il metodo moderno e corretto per le versioni recenti di CoppeliaSim.
        """
        logger.info("Setup dinamico delle collezioni via API...")

        sim = self.sim
        robot_name = self.robot_name
        try:
            # --- 1. CREA LE COLLEZIONI (o le trova se esistono già) ---
            robot_collection_handle = sim.createCollection(0)
            logger.info("  - Collezione 'Robot' creata dinamicamente.")

            fake_robot_collection_handle = sim.createCollection(0)

            env_collection_handle = sim.createCollection(0)
            logger.info("  - Collezione 'Environment' creata dinamicamente.")

            # --- 2. POPOLA LA COLLEZIONE 'ROBOT' ---
            # Aggiungiamo l'intero modello del robot (l'albero gerarchico) alla collezione.
            try:
                robot_handle = sim.getObject(f'/{robot_name}')
                # sim.handle_tree dice di includere l'oggetto e tutti i suoi figli.
                # options=0 significa "aggiungi".
                # options=2 significa escludi la base dell'albero
                sim.addItemToCollection(robot_collection_handle, sim.handle_tree, robot_handle, 2)
                logger.info(f"  - Aggiunto l'intero albero del robot '{robot_name}' alla collezione 'Robot'.")
                logger.debug(f'HANDLES DELLA COLLEZIONE ROBOT: {sim.getCollectionObjects(robot_collection_handle)}')
            except Exception as e:
                logger.exception(f"    - ERRORE: Impossibile trovare o aggiungere il robot '{robot_name}': ")

            try:
                fake_robot_handle = sim.getObject('/FakeFranka')
                sim.addItemToCollection(fake_robot_collection_handle, sim.handle_tree, fake_robot_handle, 2)
                logger.info(f"  - Aggiunto l'intero albero del robot 'Fake{robot_name}' alla collezione 'FakeRobot'.")
                logger.debug(
                    f'HANDLES DELLA COLLEZIONE FAKE ROBOT: {sim.getCollectionObjects(fake_robot_collection_handle)}')
            except Exception as e:
                logger.exception(f"    - ERRORE: Impossibile trovare o aggiungere il robot '{robot_name}': ")

            # --- 3. POPOLA LA COLLEZIONE 'ENVIRONMENT' ---
            for object_handle in environment_object_handles:
                try:
                    # sim.handle_single dice di aggiungere solo questo specifico oggetto.
                    sim.addItemToCollection(env_collection_handle, sim.handle_single, object_handle, 0)
                except Exception as e:
                    logger.exception(
                        f"    - Attenzione: Oggetto environment '{sim.getObjectAlias(object_handle)}' non trovato nella scena.")

            debug_env_collection_handles = sim.getCollectionObjects(env_collection_handle)
            debug_env_collection_names = [sim.getObjectAlias(f'{h}') for h in debug_env_collection_handles]
            logger.debug(f'NOMI DEGLI OGGETTI DELLA COLLEZIONE ENVIRONMENT: {debug_env_collection_names}')
            logger.info("✅ Collezioni create e popolate con successo.")

            return [robot_collection_handle, fake_robot_collection_handle, env_collection_handle]

        except Exception as e:
            logger.exception(f"❌ Errore critico durante la gestione delle collezioni: ")
            return None


    def perform_pick_and_hold(self, object_name):
        client, sim, simIK, simOMPL = self.client, self.sim, self.simIK, self.simOMPL
        handles = self.handles
        planner = self.planner
        actuator = self.actuator
        gripper = self.gripper

        success = False
        object_handle = sim.getObject(f'/{object_name}')
        try:
            # 1. Posizione di riposo iniziale
            # home_q = [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]
            home_q = [sim.getJointPosition(h) for h in handles['arm_joints']]

            # 2. Definizione delle pose target cartesiane relative alla base del robot
            logger.debug('CALCOLO LE POSE TARGET')
            grasp_pose = self.perception.get_grasp_pose(object_name)
            if not grasp_pose:
                logger.error("Impossibile calcolare la grasp-pose.")
                return False

            # PRE-GRASP: 15 cm sopra il bounding box
            pre_grasp_pose = list(grasp_pose)
            pre_grasp_pose[11] += 0.15

            # LIFT: Solleva di 20 cm rispetto alla presa
            lift_pose = list(grasp_pose)
            lift_pose[11] += 0.20

            # 3. Movimento verso la posa di pre-presa
            logger.debug('MI MUOVO VERSO LA PRE-PRESA')
            pre_grasp_q = planner.find_valid_ik_solution(pre_grasp_pose)
            if not pre_grasp_q:
                logger.error("Impossibile calcolare la configurazione di pre-presa.")
                return False

            path_to_pre_grasp = planner.plan_path_rrt(pre_grasp_q)
            if not path_to_pre_grasp:
                logger.error("Impossibile calcolare il percorso verso la posa di pre-presa.")
                return False
            actuator.execute_path(path_to_pre_grasp)

            # 4. Movimento di avvicinamento finale (lineare, si assume sicuro)
            logger.debug('MI ABBASSO VERTICALENTE VERSO IL CUBO')
            # setBoolProperty() SERVE PER FARE IN MODO CHE L'OGGETTO DIVENTI FIGLIO DEL GRIPPER
            # print(sim.getBoolProperty(object_handle, 'dynamic'))
            # print(sim.getBoolProperty(object_handle, 'respondable'))
            sim.setBoolProperty(object_handle, 'dynamic', False, 0)
            # sim.setBoolProperty(object_handle, 'respondable', False, 0)

            path_to_grasp = planner.plan_linear_path(grasp_pose)
            if not path_to_grasp:
                logger.error("Impossibile calcolare il percorso verso la posa di presa.")
                return False
            actuator.execute_path(path_to_grasp)
            gripper.open()

            # 5. Presa dell'oggetto
            if gripper.detect():
                gripper.attach_object(object_name)
                gripper.close()

            # 6. Sollevamento dell'oggetto
            lift_q = planner.find_valid_ik_solution(lift_pose)
            if not lift_q:
                logger.error("Impossibile calcolare la configurazione di post-presa.")
                return False
            path_to_lift = planner.plan_linear_path(lift_pose)
            if not path_to_grasp:
                logger.error("Impossibile calcolare il percorso verso la posa di post-presa.")
                return False
            actuator.execute_path(path_to_lift)

            # 7. Mantenimento della posizione per 3 secondi
            logger.debug("Mantenimento dell'oggetto in posizione...")
            # for _ in range(60):  # 60 passi * 50ms/passo = 3 secondi
            #     client.step()
            # time.sleep(3)

            # 8. Ritorno alla posizione di riposo
            path_to_home = planner.plan_path_rrt(home_q)
            if not path_to_grasp:
                logger.error("Impossibile calcolare il percorso verso la posa di Home.")
                return False
            actuator.execute_path(path_to_home)
            success = True

        finally:
            if not success:
                logger.debug("Stato dell'oggetto resettato su 'dynamic'")
                sim.setBoolProperty(object_handle, 'dynamic', True, 0)

        return success

    def perform_place(self, object_name, target_name):
        client, sim, simIK, simOMPL = self.client, self.sim, self.simIK, self.simOMPL
        handles = self.handles
        motion = self.planner
        actuator = self.actuator
        gripper = self.gripper

        # 1. Posizione di riposo iniziale
        home_q = [sim.getJointPosition(h) for h in handles['arm_joints']]

        # 2. Definizione delle pose target cartesiane relative alla base del robot
        place_pose, obj_height = self.perception.get_place_pose(object_name, target_name)
        if not place_pose:
            logger.error("Impossibile calcolare la posa di place.")
            return False

        # 2. STRATEGIA DI MOVIMENTO (Calcolo pose relative)
        # Qui applichiamo la logica di sicurezza che volevi gestire nel chiamante

        # Pre-Place: 10 cm + mezza altezza oggetto sopra la destinazione
        pre_place_pose = list(place_pose)
        pre_place_pose[11] += (obj_height / 2.0) + 0.10

        # Post-Place (Lift): Risalita di sicurezza (es. 20 cm sopra)
        lift_pose = list(place_pose)
        lift_pose[11] += 0.20

        logger.debug(f"PLACE_POSE: {place_pose}")
        logger.debug(f"PRE_PLACE_POSE: {pre_place_pose}")
        logger.debug(f"LIFT_POSE: {lift_pose}")

        # 3. Movimento verso la posa di pre-presa
        logger.debug('MI MUOVO VERSO LA PRE-PLACE')
        pre_place_q = motion.find_valid_ik_solution(pre_place_pose)
        if not pre_place_q:
            logger.error("Impossibile calcolare la configurazione di pre-place.")
            return False

        path_to_pre_place = motion.plan_path_rrt(pre_place_q)
        if not path_to_pre_place:
            logger.error("Impossibile calcolare il percorso verso la posa di pre-place.")
            return False
        actuator.execute_path(path_to_pre_place)

        # 4. Movimento di avvicinamento finale (lineare, si assume sicuro)
        logger.debug("MI ABBASSO VERTICALENTE PER POSARE L'OGGETTO")
        path_to_place = motion.plan_linear_path(place_pose)
        if not path_to_place:
            logger.error("Impossibile calcolare il percorso verso la posa di place.")
            return False
        actuator.execute_path(path_to_place)

        # 5. Rilascio dell'oggetto
        gripper.open()
        gripper.detach_object(object_name)
        # time.sleep(0.01)

        # 6. Ritorno
        logger.debug('MI SOLLEVO')
        lift_q = motion.find_valid_ik_solution(lift_pose)
        if not lift_q:
            logger.error("Impossibile calcolare la configurazione di lift.")
            return False

        path_to_lift = motion.plan_linear_path(lift_pose)
        if not path_to_lift:
            logger.error("Impossibile calcolare il percorso verso la posa di lift.")
            return False
        actuator.execute_path(path_to_lift)

        # 7. Ritorno alla posizione di riposo
        logger.debug("RITORNO ALLA POSIZIONE INIZIALE...")
        path_to_home = motion.plan_path_rrt(home_q)
        if not path_to_home:
            logger.error("Impossibile calcolare il percorso verso la posa di Home.")
            return False
        actuator.execute_path(path_to_home)

        return True

    def perform_done(self):
        print("🤖 FAKE_ESEGUO: done (compito terminato)")
        return True

    def pick_and_hold_success_detector(self, object_name):
        return self.perception.pick_and_hold_success_detector(object_name)

    def place_success_detector(self, object_name, target_name):
        return self.perception.place_success_detector(object_name, target_name)
