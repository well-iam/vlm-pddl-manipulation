from tesi_gemini_robotics.implementations.coppelia.coppeliasim_setup import *
from tesi_gemini_robotics.robot_skills import perform_pick_and_hold


# --- 2. FUNZIONI DI CONTROLLO E MOVIMENTO ---

# def control_gripper(sim, handles, action='close'):
#     """Controlla l'apertura o la chiusura del gripper."""
#     target_pos = 0.0 if action == 'close' else 0.04  # 0 = chiuso, 0.04 = aperto
#     print(f"Comando gripper: {'chiusura' if action == 'close' else 'apertura'} (target: {target_pos} m)")
#
#     for joint_handle in handles['gripper_joints']:
#         sim.setJointTargetPosition(joint_handle, target_pos)
#
#     # Attendi che il movimento del gripper si completi
#     time.sleep(1.5)

# --- 4. FUNZIONE PRINCIPALE (MAIN) ---

def main():
    """Orchestra l'intera operazione di pick and hold."""
    client, sim, simIK, simOMPL = connect_to_sim()
    if not client: return

    # 1. Recupera tutti gli handle della scena
    handles = setup_scene_handles(sim)
    if not handles: return

    # 2. Crea l'ambiente IK e ottieni gli handle tradotti
    ik_handles = setup_ik_environment(sim, simIK, handles)
    if not ik_handles: return

    # --- Inizio Simulazione ---
    sim.startSimulation()

    object_to_pick = 'Cuboid'
    success = perform_pick_and_hold(client, sim, simIK, simOMPL, handles, ik_handles, object_to_pick)



    # --- Fine Simulazione ---
    input('Premi INVIO per terminare la simulazione.')
    sim.stopSimulation()
    print("Simulazione terminata.")


if __name__ == '__main__':
    main()