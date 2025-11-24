import logging
import time
logger = logging.getLogger(__name__)

# minOpen = 25mm
# maxOpen = 105mm (range = 80mm)
# travelSpeed = 50cm/s
# forceRange = [30,70] N

class CoppeliaGripper:
    def __init__(self, sim, handles):
        self.sim = sim
        self.handles = handles
        self.proxSensor_handle = self.sim.getObject('/Franka/FrankaGripper/attachProxSensor')
        logger.info(f"acchiappato {self.sim.getObjectName(self.proxSensor_handle)}")
        time.sleep(3)

    def open(self):
        """Apre la pinza."""
        logger.debug("Apertura gripper...")
        return self._set_gripper_velocity(open_gripper=True)

    def close(self):
        """Chiude la pinza."""
        logger.debug("Chiusura gripper...")
        return self._set_gripper_velocity(open_gripper=False)

    def _set_gripper_velocity(self, open_gripper):
        """Metodo interno per azionare i motori."""
        try:
            target_velocity = 0.02 if open_gripper else -0.02
            force = 20 if open_gripper else -20

            # Scegli il secondo joint (il primo è "fittizio")
            openCloseJoint_handle = self.handles['gripper_joints'][1]
            joint_name = self.sim.getObjectAlias(openCloseJoint_handle) or self.sim.getObjectName(openCloseJoint_handle)
            logger.debug(f"Trovato giunto gripper: {joint_name}")

            logger.debug(f"[PRIMA] Velocita impostata: {self.sim.getJointTargetVelocity(openCloseJoint_handle)}")
            logger.debug(f"[PRIMA] Forza massima impostata: {self.sim.getJointTargetForce(openCloseJoint_handle)}")
            logger.debug(f"[PRIMA] Forza misurata: {self.sim.getJointForce(openCloseJoint_handle)}")
            self.sim.setJointTargetVelocity(openCloseJoint_handle, target_velocity)
            self.sim.setJointTargetForce(openCloseJoint_handle, force)
            logger.debug(f"Velocita impostata: {self.sim.getJointTargetVelocity(openCloseJoint_handle)}")
            logger.debug(f"Forza massima impostata: {self.sim.getJointTargetForce(openCloseJoint_handle)}")
            logger.debug(f"Forza misurata: {self.sim.getJointForce(openCloseJoint_handle)}")
            return True

        except Exception as e:
            logger.exception(f"Errore azionamento gripper: ")
            return False

    def detect(self):
        res, distance, _, object_handle, _ = self.sim.handleProximitySensor(self.proxSensor_handle)
        time.sleep(2)
        self.sim.resetProximitySensor(self.proxSensor_handle)
        # Considera considerare anche distance<0.03
        if res:
            obj_name = self.sim.getObjectName(object_handle)
            logger.debug(f'Rilevato oggetto "{obj_name}", a distanza {distance}')
        else:
            logger.debug(f"Non rilevato oggetti in prossimità del gripper.")

        return res

    def attach_object(self, object_name):
        """
        TRUCCO SIMULAZIONE: Rende l'oggetto figlio della pinza e aggiorna le collezioni
        per evitare auto-collisioni durante il trasporto.
        """
        try:
            logger.debug(f"Tentativo di aggancio logico (Attach) di '{object_name}'...")
            obj_handle = self.sim.getObject(f'/{object_name}')
            tip_handle = self.handles['tip']

            # 1. Check prossimità (Safety)
            # (Opzionale: verifica che l'oggetto sia davvero vicino alla pinza prima di attaccarlo)

            # 2. Set Parent
            self.sim.setObjectParent(obj_handle, tip_handle, True)  # True = KeepInPlace

            # 3. Aggiorna Collezioni (Cruciale per OMPL)
            # Rimuovi da Environment
            self.sim.addItemToCollection(self.handles['environment_collection'], self.sim.handle_single, obj_handle, 1)
            # Aggiungi a Robot
            self.sim.addItemToCollection(self.handles['robot_collection'], self.sim.handle_single, obj_handle, 0)
            # Aggiungi a Fake Robot
            self.sim.addItemToCollection(self.handles['fake_robot_collection'], self.sim.handle_single, obj_handle, 0)

            # Disabilita la dinamica per evitare jittering mentre è in mano
            self.sim.setBoolProperty(obj_handle, 'dynamic', False, 0)

            logger.info(f"Oggetto '{object_name}' attaccato alla pinza.")
            return True
        except Exception as e:
            logger.exception(f"Errore durante attach_object: ")
            return False

    def detach_object(self, object_name):
        """
        TRUCCO SIMULAZIONE: Rilascia l'oggetto e ripristina fisica/collezioni.
        """
        try:
            logger.debug(f"Tentativo di rilascio logico (Detach) di '{object_name}'...")
            obj_handle = self.sim.getObject(f'/{object_name}')

            # 1. Rimuovi Parent (diventa orfano/figlio del mondo)
            self.sim.setObjectParent(obj_handle, -1, True)

            # 2. Ripristina Collezioni
            # Rimuovi da Robot
            self.sim.addItemToCollection(self.handles['robot_collection'], self.sim.handle_single, obj_handle, 1)
            # Rimuovi da Fake Robot
            self.sim.addItemToCollection(self.handles['fake_robot_collection'], self.sim.handle_single, obj_handle, 1)
            # Aggiungi a Environment
            self.sim.addItemToCollection(self.handles['environment_collection'], self.sim.handle_single, obj_handle, 0)

            # 3. Riattiva Dinamica (così cade/si appoggia fisicamente)
            self.sim.setBoolProperty(obj_handle, 'dynamic', True, 0)

            logger.info(f"Oggetto '{object_name}' rilasciato.")
            return True
        except Exception as e:
            logger.error(f"Errore durante detach_object: {e}")
            return False