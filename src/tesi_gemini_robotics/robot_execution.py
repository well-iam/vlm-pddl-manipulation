import numpy as np
import time
import logging

logger = logging.getLogger(__name__)

def execute_path(client, sim, handles, path, tolerance=0.01, timeout=20.0):
    """Esegue un percorso definito come una lista di configurazioni di giunti."""
    logger.debug(f"Esecuzione percorso con {len(path)} waypoint...")

    for i, waypoint in enumerate(path):
        # Invia la configurazione target a tutti i giunti
        for j, joint_handle in enumerate(handles['arm_joints']):
            sim.setJointTargetPosition(joint_handle, waypoint[j])

        # Ciclo di controllo sincrono: avanza la simulazione finché il target non è raggiunto
        start_time = time.time()
        while time.time() - start_time < timeout:
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

def execute_path_OLD(client, sim, handles, path, tolerance=0.01, timeout=20.0):
    """Esegue un percorso definito come una lista di configurazioni di giunti."""
    logger.debug(f"Esecuzione percorso con {len(path)} waypoint...")

    for i, waypoint in enumerate(path):
        # Invia la configurazione target a tutti i giunti

        for j, joint_handle in enumerate(handles['arm_joints']):
            # print(f'PRIMA ({j}): {sim.getJointTargetPosition(joint_handle)}')
            sim.setJointTargetPosition(joint_handle, waypoint[j])
            # print(f'DOPO ({j}): {sim.getJointTargetPosition(joint_handle)}')

        # Ciclo di controllo sincrono: avanza la simulazione finché il target non è raggiunto
        while True:
            # client.step()  # Avanza di un passo di simulazione

            current_q = [sim.getJointPosition(h) for h in handles['arm_joints']]
            # Calcola la distanza (norma) tra la configurazione attuale e quella target
            dist = np.linalg.norm(np.array(current_q) - np.array(waypoint))
            # print(f'dist={dist}') #DEBUG
            if dist < tolerance:
                # print(f"Waypoint {i+1}/{len(path)} raggiunto.")
                break
    logger.debug("✅ Percorso completato.")