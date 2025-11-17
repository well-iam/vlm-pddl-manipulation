from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import time


def setup_robot_handles(sim):
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
        handles['gripper'] = sim.getObject('/Franka/FrankaGripper')
        handles['target'] = sim.getObject('/Franka_target')
        handles['tip'] = sim.getObject('/Franka/connection')  # Cerca il punto di connessione (end-effector)
        handles['cuboid'] = sim.getObject('/Cuboid')


        # 2. TROVARE I GIUNTI IN MODO DINAMICO (LA PARTE CHIAVE)
        # Invece di cercare 'joint1', 'joint2', ecc., chiediamo al simulatore di darci
        # tutti gli oggetti di tipo "giunto" che sono figli del modello Franka.

        all_robot_joints = sim.getObjectsInTree(handles['base'], sim.object_joint_type, 0)

        # Ora separiamo i giunti del braccio da quelli della pinza.
        # Per farlo, troviamo tutti i giunti che sono figli del modello 'FrankaGripper'.
        gripper_joints_handles = sim.getObjectsInTree(handles['gripper'], sim.object_joint_type, 0)

        # I giunti del braccio sono tutti i giunti del robot, escludendo quelli della pinza.
        arm_joints_handles = [h for h in all_robot_joints if h not in gripper_joints_handles]

        handles['joints'] = arm_joints_handles
        handles['gripper_joints'] = gripper_joints_handles

        

        # Riepilogo per verifica
        print(f"  - Trovato robot: '{sim.getObjectAlias(handles['base'])}'")
        print(f"  - Trovati {len(handles['joints'])} giunti del braccio.")
        print(f"  - Trovati {len(handles['gripper_joints'])} giunti della pinza.")
        print(f"  - Trovato target IK: '{sim.getObjectAlias(handles['target'])}'")

    except Exception as e:
        print(f"❌ ERRORE durante il recupero di un handle: {e}")
        print("    Controlla che tutti gli oggetti principali (es. /Franka, /FrankaGripper, /Franka_target, /Cuboid) "
              "siano presenti e abbiano i nomi corretti nella scena.")
        return None

    print("✅ Handle recuperati con successo.")
    return handles


def generate_joint_space_trajectory(q_start, q_goal, duration, dt):
    """
    Genera una traiettoria nello spazio dei giunti tramite interpolazione lineare.

    Args:
        q_start (list or np.array): Configurazione iniziale dei giunti (7 valori).
        q_goal (list or np.array): Configurazione finale dei giunti (7 valori).
        duration (float): Durata totale del movimento in secondi.
        dt (float): Passo temporale di simulazione in secondi.

    Returns:
        list: Una lista di configurazioni di giunti (liste di 7 valori) che rappresentano la traiettoria.
    """
    q_start = np.array(q_start)
    q_goal = np.array(q_goal)
    num_steps = int(duration / dt)
    trajectory = []

    for i in range(num_steps + 1):
        alpha = i / num_steps
        q_current = (1 - alpha) * q_start + alpha * q_goal
        trajectory.append(q_current.tolist())

    return trajectory

if __name__ == '__main__':
    # --- Connessione al server ZMQ di CoppeliaSim ---
    client = RemoteAPIClient()
    sim = client.require('sim')

    # --- Avvio della simulazione ---
    sim.setStepping(True)  # Abilita la modalità sincrona
    sim.startSimulation()
    print("Simulazione avviata.")

    # Ottieni handles
    handles = setup_robot_handles(sim)
    joint_handles = handles['joints']
    # print(f'PRIMO: {joint_handles}') #DEBUG handles dei giunti

    # --- Definizione della traiettoria ---
    # Ottieni la configurazione iniziale attuale del robot
    q_start = [sim.getJointPosition(h) for h in joint_handles]
    # print(f'START: {q_start}')

    # Definisci una configurazione finale (in radianti)
    q_goal = [d*3.14/180 for d in [30, 60, 30, -60, 30, 90, 0]]
    # print(f'GOAL: {q_goal}')

    duration = 5.0  # secondi
    dt = sim.getSimulationTimeStep()  # Ottieni il dt dalla simulazione
    # print(f'dt= {dt}') #DEBUG

    trajectory = generate_joint_space_trajectory(q_start, q_goal, duration, dt)


    # --- Esecuzione della traiettoria ---
    print(f"Esecuzione della traiettoria (len={len(trajectory)}) nello spazio dei giunti...")
    for q_target in trajectory:
        for i in range(7):
            # Imposta la posizione target per ogni giunto
            sim.setJointPosition(joint_handles[i], q_target[i])
        sim.step()  # Avanza la simulazione di un passo


    #time.sleep(10)
    print("Traiettoria completata.")
    time.sleep(2)  # Pausa prima di terminare

    # --- Cleanup ---
    sim.stopSimulation()
    print("Simulazione terminata.")