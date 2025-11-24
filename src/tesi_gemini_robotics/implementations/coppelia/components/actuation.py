import numpy as np
import time
import logging

logger = logging.getLogger(__name__)

class CoppeliaActuator:
    def __init__(self, client, sim, handles):
        self.client = client
        self.sim = sim
        self.handles = handles

    def execute_path(self, path, tolerance=0.01, timeout=20.0):
        """Esegue un percorso definito come una lista di configurazioni di giunti."""
        logger.debug(f"Esecuzione percorso con {len(path)} waypoint...")
        sim = self.sim
        handles = self.handles

        for i, waypoint in enumerate(path):
            # Invia la configurazione target a tutti i giunti
            for j, joint_handle in enumerate(handles['arm_joints']):
                sim.setJointTargetPosition(joint_handle, waypoint[j])

            # Ciclo di controllo sincrono: avanza la simulazione finché il target non è raggiunto
            start_time = time.time()
            while time.time() - start_time < timeout:
                # client.step()  # Avanza di un passo di simulazione
                current_q = [sim.getJointPosition(h) for h in handles['arm_joints']]
                # Calcola la distanza (norma) tra la configurazione attuale e quella target
                dist = np.linalg.norm(np.array(current_q) - np.array(waypoint))
                # print(f'dist={dist}') #DEBUG
                if dist < tolerance:
                    #print(f"Waypoint {i+1}/{len(path)} raggiunto.")
                    break
                # Pausa per non sovraccaricare la CPU
                time.sleep(0.05)
            else:
                logger.warning(f"  - ⚠️ TIMEOUT raggiunto per il waypoint {i + 1}.")
                return False

        logger.debug("✅ Percorso completato.")
        return True