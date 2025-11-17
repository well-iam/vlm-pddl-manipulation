import logging
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

logger = logging.getLogger(__name__)

def connect_to_sim(host='localhost', port=23000):
    """Stabilisce la connessione con CoppeliaSim e imposta la modalità sincrona."""
    logger.debug("Tentativo di connessione a CoppeliaSim...")
    client = RemoteAPIClient(host, port)
    # client.setStepping(True)
    try:
        sim = client.require('sim')
        simIK = client.require('simIK')
        simOMPL = client.require('simOMPL')
        logger.info("✅ Connessione a CoppeliaSim e ai moduli IK/OMPL stabilita.")
        return client, sim, simIK, simOMPL
    except Exception as e:
        logger.exception("❌ Connessione fallita:")
        return None, None, None, None

# TODO: GRIPPER
def setup_scene_handles(sim):
    """
    Ottiene e restituisce gli handle per i componenti chiave del robot e della scena
    in modo robusto, cercando gli oggetti per tipo e gerarchia.
    """
    logger.debug("Recupero degli handle degli oggetti in modo robusto...")
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
        # handles['cuboid'] = sim.getObject('/Red_Cuboid')

        # 2. TROVARE I GIUNTI IN MODO DINAMICO (LA PARTE CHIAVE)
        # Invece di cercare 'joint1', 'joint2', ecc., chiediamo al simulatore di darci
        # tutti gli oggetti di tipo "giunto" che sono figli del modello Franka.

        all_robot_joints = sim.getObjectsInTree(handles['base'], sim.object_joint_type, 0)

        joint_names = [
            f"  - Nome: '{sim.getObjectAlias(h) or sim.getObjectName(h)}', Handle: {h}"
            for h in all_robot_joints
        ]
        joint_list_string = "\n".join(joint_names)
        logger.debug(f"""
        Trovati {len(all_robot_joints)} oggetti di tipo 'joint' in totale sotto '/Franka':
        {joint_list_string}""")

        gripper_joints_handles = sim.getObjectsInTree(handles['gripper'], sim.object_joint_type, 0)
        arm_joints_handles = [h for h in all_robot_joints if h not in gripper_joints_handles]

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
        ignore_list = [handles['base'], handles['fake_base'], sim.getObject('/Floor')]
        env_objects_handles = [h for h in all_top_level_shape_handles if h not in ignore_list]
        logger.debug(all_top_level_shape_handles)
        logger.debug(env_objects_handles)

        # Chiama la nuova funzione per creare le collezioni
        collection_handles = setup_collections_via_api(sim, robot_name='Franka', environment_object_handles=env_objects_handles)
        if not collection_handles:
            logger.critical("Impossibile creare le collezioni, termino il programma.")
            return

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
        logger.error("    Controlla che tutti gli oggetti principali (es. /Franka, /FrankaGripper, /Franka_target, /Cuboid) "
              "siano presenti e abbiano i nomi corretti nella scena.")
        return None

    logger.info("✅ Handle recuperati con successo.")
    return handles

def setup_collections_via_api(sim, robot_name, environment_object_handles):
    """
    Crea e popola le collezioni 'Robot' e 'Environment' dinamicamente via API.
    Questo è il metodo moderno e corretto per le versioni recenti di CoppeliaSim.
    """
    logger.info("Setup dinamico delle collezioni via API...")

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
            sim.addItemToCollection(robot_collection_handle, sim.handle_tree, robot_handle, 0)
            logger.info(f"  - Aggiunto l'intero albero del robot '{robot_name}' alla collezione 'Robot'.")
            logger.debug(f'HANDLES DELLA COLLEZIONE ROBOT: {sim.getCollectionObjects(robot_collection_handle)}')
        except Exception as e:
            logger.exception(f"    - ERRORE: Impossibile trovare o aggiungere il robot '{robot_name}': ")

        try:
            fake_robot_handle = sim.getObject('/FakeFranka')
            # sim.handle_tree dice di includere l'oggetto e tutti i suoi figli.
            # options=0 significa "aggiungi".
            sim.addItemToCollection(fake_robot_collection_handle, sim.handle_tree, fake_robot_handle, 0)
            logger.info(f"  - Aggiunto l'intero albero del robot 'Fake{robot_name}' alla collezione 'FakeRobot'.")
            logger.debug(f'HANDLES DELLA COLLEZIONE FAKE ROBOT: {sim.getCollectionObjects(fake_robot_collection_handle)}')
        except Exception as e:
            logger.exception(f"    - ERRORE: Impossibile trovare o aggiungere il robot '{robot_name}': ")

        # --- 3. POPOLA LA COLLEZIONE 'ENVIRONMENT' ---
        for object_handle in environment_object_handles:
            try:
                # sim.handle_single dice di aggiungere solo questo specifico oggetto.
                sim.addItemToCollection(env_collection_handle, sim.handle_single, object_handle, 0)
            except Exception as e:
                logger.exception(f"    - Attenzione: Oggetto environment '{sim.getObjectAlias(object_handle)}' non trovato nella scena.")

        logger.debug(f'HANDLES DELLA COLLEZIONE ENVIRONMENT: {sim.getCollectionObjects(env_collection_handle)}')
        logger.info("✅ Collezioni create e popolate con successo.")

        return [robot_collection_handle, fake_robot_collection_handle, env_collection_handle]

    except Exception as e:
        logger.exception(f"❌ Errore critico durante la gestione delle collezioni: ")
        return None

def setup_ik_environment(sim, simIK, scene_handles):
    """
    Crea l'ambiente IK e TRADUCE gli handle della scena in handle IK.
    """
    logger.info("Setup dell'ambiente di Cinematica Inversa (IK)...")
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