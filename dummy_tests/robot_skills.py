from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import numpy as np


# --- FUNZIONI HELPER DI CONTROLLO (leggermente migliorate) ---

def move_arm_to_position(sim, ik_target_handle, position, orientation_handle=None):
    """
    Muove il braccio robotico a una posizione specifica.
    Se orientation_handle è fornito, mantiene l'orientamento di quell'oggetto.
    """
    print(f"    -> Muovendo il braccio a {np.round(position, 2)}...")

    if orientation_handle:
        orientation = sim.getObjectQuaternion(orientation_handle, -1)
        sim.setObjectQuaternion(ik_target_handle, -1, orientation)

    sim.setObjectPosition(ik_target_handle, -1, position)
    time.sleep(2)  # Pausa per dare al robot il tempo di muoversi


def set_gripper(sim, gripper_joint_handle, open_gripper):
    """
    Apre (True) o chiude (False) la pinza.
    NOTA: Questi valori di velocità potrebbero dover essere adattati al tuo modello di pinza.
    """
    target_velocity = -0.04 if open_gripper else 0.04
    motor_force = 50

    # Imposta la velocità e la forza del motore del giunto della pinza
    sim.setJointTargetVelocity(gripper_joint_handle, target_velocity)
    sim.setJointTargetForce(gripper_joint_handle, motor_force)

    time.sleep(1.5)  # Pausa per l'azione della pinza
    sim.setJointTargetVelocity(gripper_joint_handle, 0)  # Ferma il motore


# --- LA NUOVA SKILL: PICK AND HOLD ---

def pick_and_hold(sim, object_to_pick_name):
    """
    Esegue un'azione completa di pick and hold su un oggetto specifico.
    """
    print(f"\nInizio skill: pick_and_hold su '{object_to_pick_name}'")

    try:
        # 1. Ottieni gli "handle" degli oggetti chiave nella scena
        ik_target_handle = sim.getObject('/Franka_target')  # Il target per l'IK del braccio
        gripper_handle = sim.getObject('/Franka/FrankaGripper/openCloseJoint')  # Il giunto motore della pinza
        gripper_tip_handle = sim.getObject('/Franka/FrankaGripper/attachPoint')  # Il punto di aggancio sulla punta del braccio
        object_handle = sim.getObject(f'/{object_to_pick_name}')

        # 2. Posizione di approccio (10cm sopra l'oggetto)
        object_pos = sim.getObjectPosition(object_handle, -1)
        approach_pos = [object_pos[0], object_pos[1], object_pos[2] + 0.10]

        # 3. Muovi sopra l'oggetto e apri la pinza
        print("  FASE 1: Avvicinamento")
        move_arm_to_position(sim, ik_target_handle, approach_pos, orientation_handle=object_handle)
        set_gripper(sim, gripper_handle, open_gripper=True)

        # 4. Scendi sull'oggetto
        move_arm_to_position(sim, ik_target_handle, object_pos, orientation_handle=object_handle)

        # 5. Chiudi la pinza per afferrare fisicamente
        print("  FASE 2: Chiusura pinza")
        set_gripper(sim, gripper_handle, open_gripper=False)

        # 6. "Saldatura" digitale: rendi l'oggetto un figlio della punta della pinza
        print("  FASE 3: Aggancio (Hold)")
        sim.setObjectParent(object_handle, gripper_tip_handle, keepInPlace=True)

        # 7. Solleva l'oggetto per verificare la presa
        print("  FASE 4: Sollevamento")
        move_arm_to_position(sim, ik_target_handle, approach_pos)

        print(f"✅ Skill completata. L'oggetto '{object_to_pick_name}' è ora 'tenuto' dalla pinza.")
        return True

    except Exception as e:
        print(f"❌ Errore durante l'esecuzione di pick_and_hold: {e}")
        return False


# --- ESECUZIONE MANUALE DEL TEST ---
if __name__ == '__main__':
    client = RemoteAPIClient()
    sim = client.require('sim')

    try:
        # Assicurati che la simulazione sia in esecuzione in CoppeliaSim
        if sim.getSimulationState() == sim.simulation_stopped:
            sim.startSimulation()
            time.sleep(1)

        # Esegui la skill sull'oggetto che ti interessa
        success = pick_and_hold(sim, object_to_pick_name='Cuboid')

        if success:
            print("\nTest di movimento: sposto l'oggetto a destra...")
            # Ottieni la posizione corrente e spostala
            current_pos = sim.getObjectPosition(sim.getObject('/Franka_target'), -1)
            new_pos = [current_pos[0], current_pos[1] - 0.2, current_pos[2]]  # Sposta di 20cm sull'asse Y
            move_arm_to_position(sim, sim.getObject('/Franka_target'), new_pos)
            print("Verifica completata.")

    finally:
        # Lasciamo la simulazione in esecuzione per osservare il risultato
        # sim.stopSimulation()
        print("\nScript terminato. L'oggetto dovrebbe essere in mano al robot.")