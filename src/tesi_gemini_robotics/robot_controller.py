from tesi_gemini_robotics.coppeliasim_setup import *
from tesi_gemini_robotics.motion_planning import *
from tesi_gemini_robotics.robot_execution import *
from tesi_gemini_robotics.utils import dist_xy, dist, is_on

class RobotController:
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
        self.client = client
        self.sim = sim
        self.simIK = simIK
        self.simOMPL = simOMPL
        self.robot_name = robot_name

        # Mappa Nomi Skill -> Funzioni Python
        # Questo dizionario collega il nome che Gemini sceglierà alla funzione da eseguire.
        self.available_skills = {
            "pick_and_hold": self.perform_pick_and_hold,
            "place": self.perform_place,
            "pick_and_place": self.perform_pick_and_place,
            "done": self.perform_done
        }
        self.available_detectors = {
            "pick_and_hold": self.pick_and_hold_success_detector,
            "place": self.place_success_detector,
        }

        # 1. Recupera Handles della Scena
        self.handles = setup_scene_handles(self.sim)
        if not self.handles:
            raise RuntimeError("Setup fallito: impossibile recuperare gli handle della scena.")

        # 2. Setup Ambiente IK
        self.ik_handles = setup_ik_environment(self.sim, self.simIK, self.handles)
        if not self.ik_handles:
            raise RuntimeError("Setup fallito: impossibile creare l'ambiente IK.")

        logger.debug(f"✅ Skill disponibili per il robot: {list(self.available_skills.keys())}")
        logger.info("✅ RobotController inizializzato con successo.")


    # def perform_pick_and_hold(client, sim, simIK, simOMPL, handles, ik_handles, object_name):
    def perform_pick_and_hold(self, object_name):
        client, sim, simIK, simOMPL = self.client, self.sim, self.simIK, self.simOMPL
        handles, ik_handles = self.handles, self.ik_handles

        success = False
        object_handle = sim.getObject(f'/{object_name}')
        try:
            # 1. Posizione di riposo iniziale
            # home_q = [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]
            home_q = [sim.getJointPosition(h) for h in handles['arm_joints']]

            # 2. Definizione delle pose target cartesiane relative alla base del robot
            object_pose = sim.getObjectMatrix(object_handle, handles['base'])

            # Posa di pre-presa: 10 cm sopra il cubo
            _, _, object_dimensions = sim.getShapeGeomInfo(object_handle)
            object_height = object_dimensions[0]
            pre_grasp_pose = list(object_pose)
            pre_grasp_pose[11] += 2 * object_height  # z_index=11

            # Posa di presa
            grasp_pose = object_pose
            grasp_pose[11] += 0.5 * object_height

            # Posa di sollevamento
            lift_pose = list(grasp_pose)
            lift_pose[11] += 3 * object_height

            # 3. Movimento verso la posa di pre-presa
            logger.debug('MI MUOVO VERSO LA PRE-PRESA')
            pre_grasp_q = find_valid_ik_solution(sim, simIK, ik_handles, handles, pre_grasp_pose)
            if not pre_grasp_q:
                logger.error("Impossibile calcolare la configurazione di pre-presa.")
                return False

            path_to_pre_grasp = plan_path_rrt(sim, simOMPL, handles, pre_grasp_q)
            if not path_to_pre_grasp:
                logger.error("Impossibile calcolare il percorso verso la posa di pre-presa.")
                return False
            execute_path(client, sim, handles, path_to_pre_grasp)


            # 4. Movimento di avvicinamento finale (lineare, si assume sicuro)
            logger.debug('MI ABBASSO VERTICALENTE VERSO IL CUBO')
            # time.sleep(3)
            # grasp_q = solve_ik(simIK, ik_handles, grasp_pose)
            # setBoolProperty() SERVE PER FARE IN MODO CHE L'OGGETTO DIVENTI FIGLIO DEL GRIPPER
            # print(sim.getBoolProperty(object_handle, 'dynamic'))
            # print(sim.getBoolProperty(object_handle, 'respondable'))
            sim.setBoolProperty(object_handle, 'dynamic', False, 0)
            # sim.setBoolProperty(object_handle, 'respondable', False, 0)

            # print(sim.getBoolProperty(object_handle, 'dynamic'))
            # print(sim.getBoolProperty(object_handle, 'respondable'))
            # time.sleep(5)

            path_to_grasp = solve_linear_path(simIK, ik_handles, grasp_pose)
            if not path_to_grasp:
                logger.error("Impossibile calcolare il percorso verso la posa di presa.")
                return False
            execute_path(client, sim, handles, path_to_grasp)

            # 5. Presa dell'oggetto
            # control_gripper(sim, handles, 'close')
            xy_dist = dist_xy(sim, object_handle, handles['tip'])
            if xy_dist < 0.03:
                sim.setObjectParent(object_handle, handles['tip'], True)  # KeepInPlace=True
                sim.addItemToCollection(handles['environment_collection'], sim.handle_single, object_handle, 1)
                sim.addItemToCollection(handles['robot_collection'], sim.handle_single, object_handle, 0)
                # sim.addItemToCollection(handles['fake_robot_collection'], sim.handle_single, object_handle, 0)

            # 6. Sollevamento dell'oggetto
            lift_q = find_valid_ik_solution(sim, simIK, ik_handles, handles, lift_pose)
            if not lift_q:
                logger.error("Impossibile calcolare la configurazione di post-presa.")
                return False
            path_to_lift = solve_linear_path(simIK, ik_handles, lift_pose)
            if not path_to_grasp:
                logger.error("Impossibile calcolare il percorso verso la posa di post-presa.")
                return False
            execute_path(client, sim, handles, path_to_lift)

            # 7. Mantenimento della posizione per 3 secondi
            logger.debug("Mantenimento dell'oggetto in posizione...")
            # for _ in range(60):  # 60 passi * 50ms/passo = 3 secondi
            #     client.step()
            # time.sleep(3)

            # 8. Ritorno alla posizione di riposo
            path_to_home = plan_path_rrt(sim, simOMPL, handles, home_q)
            if not path_to_grasp:
                logger.error("Impossibile calcolare il percorso verso la posa di Home.")
                return False
            execute_path(client, sim, handles, path_to_home)
            success = True

        finally:
            if not success:
                logger.debug("Stato dell'oggetto resettato su 'dynamic'")
                sim.setBoolProperty(object_handle, 'dynamic', True, 0)

        return success

    def perform_place(self, object_name, target_name):
        client = self.client
        sim = self.sim
        simIK = self.simIK
        simOMPL = self.simOMPL
        handles = self.handles
        ik_handles = self.ik_handles
        # 1. Posizione di riposo iniziale
        home_q = [sim.getJointPosition(h) for h in handles['arm_joints']]

        # 2. Definizione delle pose target cartesiane relative alla base del robot
        object_handle = sim.getObject(f'/{object_name}')
        target_handle = sim.getObject(f'/{target_name}')
        target_pose = sim.getObjectMatrix(target_handle, handles['base'])

        # Posa di pre-place: 10 cm sopra il pad
        _, _, object_dimensions = sim.getShapeGeomInfo(object_handle)
        _, _, target_dimensions = sim.getShapeGeomInfo(target_handle)
        target_height = target_dimensions[2]
        object_height = object_dimensions[2]
        logger.debug(f"TARGET_HEIGHT:{target_height}")
        logger.debug(f"OBJECT_HEIGHT:{object_height}")
        pre_place_pose = list(target_pose)
        logger.debug(f"PRE_PLACE_POSE z: {pre_place_pose[11]}")
        pre_place_pose[11] += target_height + 2*object_height  # z_index=11
        logger.debug(f"PRE_PLACE_POSE z: {pre_place_pose[11]}")

        # Posa di place
        place_pose = list(pre_place_pose) #FONDAMENTALE il list(), perché sennò modificherei pre_place_pos
        place_pose[11] -= 0.5*object_height

        # Posa di sollevamento
        lift_pose = list(pre_place_pose)
        # lift_pose[11] += 3 * object_height

        # 3. Movimento verso la posa di pre-presa
        logger.debug('MI MUOVO VERSO LA PRE-PLACE')
        pre_place_q = find_valid_ik_solution(sim, simIK, ik_handles, handles, pre_place_pose)
        if not pre_place_q:
            logger.error("Impossibile calcolare la configurazione di pre-place.")
            return False

        path_to_pre_place = plan_path_rrt(sim, simOMPL, handles, pre_place_q)
        if not path_to_pre_place:
            logger.error("Impossibile calcolare il percorso verso la posa di pre-place.")
            return False
        execute_path(client, sim, handles, path_to_pre_place)

        # 4. Movimento di avvicinamento finale (lineare, si assume sicuro)
        logger.debug("MI ABBASSO VERTICALENTE PER POSARE L'OGGETTO")
        # time.sleep(3)
        # grasp_q = solve_ik(simIK, ik_handles, grasp_pose)
        path_to_place = solve_linear_path(simIK, ik_handles, place_pose)
        if not path_to_place:
            logger.error("Impossibile calcolare il percorso verso la posa di place.")
            return False
        execute_path(client, sim, handles, path_to_place)

        # 5. Rilascio dell'oggetto
        # control_gripper(sim, handles, 'close')
        sim.addItemToCollection(handles['robot_collection'], sim.handle_single, object_handle, 1)
        # sim.addItemToCollection(handles['fake_robot_collection'], sim.handle_single, object_handle, 1)
        sim.addItemToCollection(handles['environment_collection'], sim.handle_single, object_handle, 0)
        sim.setObjectParent(object_handle, -1, True)  # KeepInPlace=True

        # setBoolProperty() SERVE PER FARE IN MODO CHE L'OGGETTO DIVENTI FIGLIO DEL GRIPPER
        sim.setBoolProperty(object_handle, 'dynamic', True, 0)
        # sim.setBoolProperty(object_handle, 'respondable', True, 0)
        # time.sleep(0.01)

        # 6. Ritorno
        logger.debug('MI SOLLEVO')
        lift_q = find_valid_ik_solution(sim, simIK, ik_handles, handles, lift_pose)
        if not lift_q:
            logger.error("Impossibile calcolare la configurazione di lift.")
            return False

        path_to_lift = solve_linear_path(simIK, ik_handles, lift_pose)
        if not path_to_place:
            logger.error("Impossibile calcolare il percorso verso la posa di lift.")
            return False
        execute_path(client, sim, handles, path_to_lift)

        # 7. Ritorno alla posizione di riposo
        logger.debug("RITORNO ALLA POSIZIONE INIZIALE...")
        path_to_home = plan_path_rrt(sim, simOMPL, handles, home_q)
        if not path_to_home:
            logger.error("Impossibile calcolare il percorso verso la posa di Home.")
            return False
        execute_path(client, sim, handles, path_to_home)

        return True

    def perform_pick_and_place(self, object_name, location_name):
        print(f"🤖 FAKE_ESEGUO: pick_and_place di '{object_name}' su '{location_name}'")
        # ...
        return True

    def perform_done(self):
        print("🤖 FAKE_ESEGUO: done (compito terminato)")
        return True

    def pick_and_hold_success_detector(self, object_name):
        object_handle = self.sim.getObject(f'/{object_name}')

        if dist(self.sim, object_handle, self.handles['tip']) > 0.05:
            return False, "Oggetto non afferrato"
        return True, "True"

    def place_success_detector(self, object_name, target_name):
        object_handle = self.sim.getObject(f'/{object_name}')
        target_handle = self.sim.getObject(f'/{target_name}')

        if is_on(self.sim, object_handle, target_handle):
            return True, "True"
        return False, "Oggetto non lasciato su target"
