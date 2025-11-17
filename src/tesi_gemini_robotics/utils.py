import math
import json
import numpy as np
from PIL import Image
import logging
import time

logger = logging.getLogger(__name__)

def get_image_of_scene(sim):
    try:
        vision_sensor_handle = sim.getObject('/visionSensor')
        img, res = sim.getVisionSensorImg(vision_sensor_handle)
        img_np = np.frombuffer(img, dtype=np.uint8).reshape((res[1], res[0], 3))
        img_np = np.flipud(img_np)
        pil_image = Image.fromarray(img_np, 'RGB')
        pil_image.save("scatto_simulatore.png")
    except Exception as e:
        logger.exception(f"❌ Errore nell'acquisizione dell'immagine: {e}")
        return None
    return pil_image

def get_top_level_manipulable_objects_handle(sim, ignore_list=['Floor']):
    """

    Interroga la scena di CoppeliaSim per identificare tutti gli oggetti
    di primo livello (figli diretti della scena) che sono sia dinamici
    che interagenti (respondable).

    Questo metodo utilizza l'approccio corretto e più efficiente, sfruttando
    il parametro 'options' della funzione sim.getObjectsInTree per delegare
    il filtraggio gerarchico al motore di simulazione.

    :param sim: L'oggetto client dell'API di CoppeliaSim.
    :return: Una lista di stringhe contenente i nomi degli oggetti che
             soddisfano tutti i criteri.
    """
    logger.debug("Recupero degli oggetti interattivi di primo livello (Metodo Ottimizzato)...")
    object_names = []

    # --- LA LOGICA CORRETTA BASATA SULLA RICERCA DOCUMENTALE ---

    # 1. Ottiene gli handle di TUTTI gli oggetti di primo livello nella scena.
    #    - treeBaseHandle = sim.handle_scene: La ricerca parte dalla radice della scena.
    #    - objectType = sim.handle_all: La ricerca è generalizzata a qualsiasi
    #      tipo di oggetto per massima robustezza (non solo 'shape').
    #    - options = 2: QUESTA È LA CHIAVE. Il valore 2 (bit 1 impostato)
    #      istruisce la funzione a restituire SOLO i figli diretti della radice,
    #      escludendo tutti i sotto-oggetti in gerarchie più profonde.
    all_top_level_objects = sim.getObjectsInTree(sim.handle_scene, sim.sceneobject_shape, 2)  # sim.handle_all
    logger.debug(f"Trovati {len(all_top_level_objects)} oggetti totali di primo livello nella scena.")

    # ----------------------------------------------------------------

    for handle in all_top_level_objects:
        name = ""  # Inizializza il nome per l'uso nel blocco except
        try:
            # Recupera il nome più user-friendly: l'alias se esiste, altrimenti il nome dell'oggetto.
            # Questa è una best practice per l'identificazione degli oggetti.[4]
            name = sim.getObjectAlias(handle) or sim.getObjectName(handle)
            if name in ignore_list:
                continue

            # Filtraggio delle proprietà
            is_dynamic = sim.getBoolProperty(handle, 'dynamic')
            is_respondable = sim.getBoolProperty(handle, 'respondable')

            # print(f"\nAnalizzo oggetto: '{name}' (Handle: {handle})")
            # print(f"  - È Dynamic?   -> {is_dynamic}")
            # print(f"  - È Respondable? -> {is_respondable}")

            if is_dynamic and is_respondable:
                logger.debug(f"  --> ✅ CONDIZIONI SODDISFATTE. Aggiungo '{name}' alla lista.")
                object_names.append(name)
            else:
                logger.debug(f"  --> ❌ CONDIZIONI NON SODDISFATTE. Ignoro l'oggetto.")

        except Exception as e:
            # Un blocco try-except è fondamentale qui. Poiché cerchiamo oggetti
            # di tipo 'sim.handle_all', potremmo incontrare oggetti (es. script,
            # collezioni) che non supportano le proprietà 'dynamic' o 'respondable'.
            # Questo blocco previene il crash dello script e gestisce l'errore.
            object_identity = f"'{name}'" if name else f"con handle {handle}"
            logger.exception(f"\n  --> ⚠️ ERRORE o oggetto non pertinente {object_identity}: {e}")

    print(f"\n✅ Trovati {len(object_names)} oggetti manipolabili di primo livello: {object_names}")
    return object_names

def get_top_level_objects(sim, ignore_list):
        """

        Interroga la scena di CoppeliaSim per identificare tutti gli oggetti
        di primo livello (figli diretti della scena).

        Questo metodo utilizza l'approccio corretto e più efficiente, sfruttando
        il parametro 'options' della funzione sim.getObjectsInTree per delegare
        il filtraggio gerarchico al motore di simulazione.

        :param sim: L'oggetto client dell'API di CoppeliaSim.
        :return: Una lista di stringhe contenente i nomi degli oggetti che
                 soddisfano tutti i criteri.
        """
        logger.debug("Recupero degli oggetti interattivi di primo livello (Metodo Ottimizzato)...")
        object_names = []
        object_handles = []

        # --- LA LOGICA CORRETTA BASATA SULLA RICERCA DOCUMENTALE ---

        # 1. Ottiene gli handle di TUTTI gli oggetti di primo livello nella scena.
        #    - treeBaseHandle = sim.handle_scene: La ricerca parte dalla radice della scena.
        #    - objectType = sim.handle_all: La ricerca è generalizzata a qualsiasi
        #      tipo di oggetto per massima robustezza (non solo 'shape').
        #    - options = 2: QUESTA È LA CHIAVE. Il valore 2 (bit 1 impostato)
        #      istruisce la funzione a restituire SOLO i figli diretti della radice,
        #      escludendo tutti i sotto-oggetti in gerarchie più profonde.
        all_top_level_objects = sim.getObjectsInTree(sim.handle_scene, sim.sceneobject_shape, 2)  # sim.handle_all
        logger.debug(f"Trovati {len(all_top_level_objects)} oggetti totali di primo livello nella scena.")

        # ----------------------------------------------------------------

        for handle in all_top_level_objects:
            name = ""  # Inizializza il nome per l'uso nel blocco except
            try:
                # Recupera il nome più user-friendly: l'alias se esiste, altrimenti il nome dell'oggetto.
                # Questa è una best practice per l'identificazione degli oggetti.[4]
                name = sim.getObjectAlias(handle) or sim.getObjectName(handle)
                if name in ignore_list:
                    continue
                object_handles.append(handle)
                object_names.append(name)

            except Exception as e:
                object_identity = f"'{name}'" if name else f"con handle {handle}"
                logger.exception(f"\n  --> ⚠️ ERRORE o oggetto non pertinente {object_identity}: ")

        logger.debug(f"""
        ✅ Trovati {len(object_names)} oggetti di primo livello: {object_names}
        ✅ Trovati {len(object_handles)} oggetti di primo livello: {object_handles}"))
        """)
        return object_names, object_handles


def get_object_positions_from_sim(sim, object_names, robot_name):
    """
    FASE 2: Prende la lista di nomi e interroga CoppeliaSim per le coordinate 3D.
    Restituisce un dizionario con le posizioni e la posizione del robot.
    """
    logger.debug("\n--- FASE 2: Arricchimento Dati (CoppeliaSim) ---")
    object_positions = {}

    for name in object_names:
        try:
            # Aggiungiamo il '/' se non è presente per la ricerca nella gerarchia
            full_name = name if name.startswith('/') else f'/{name}'
            handle = sim.getObject(full_name)

            # -1 indica che vogliamo le coordinate rispetto al frame del mondo
            position = sim.getObjectPosition(handle, -1)
            object_positions[name] = position
            logger.debug(f"✅ Posizione di '{name}': {np.round(position, 2)}")
        except Exception:
            logger.warning(f"⚠️ Attenzione: Gemini ha menzionato '{name}', ma non è stato trovato nella scena di CoppeliaSim.")

    # Ottieni anche la posizione della base del robot
    try:
        full_robot_name = robot_name if robot_name.startswith('/') else f'/{robot_name}'
        robot_handle = sim.getObject(f'{full_robot_name}')
        robot_position = sim.getObjectPosition(robot_handle, -1)
        logger.debug(f"✅ Posizione del robot '{robot_name}': {np.round(robot_position, 2)}")
    except Exception:
        logger.exception(f"❌ Errore: l'oggetto base del robot '{robot_name}' non è stato trovato. Controlla il nome.")
        robot_position = None

    return object_positions, robot_position


def analyze_reachability(robot_pos, object_pos, max_reach=0.855):
    """
    Calcola la distanza e determina se un oggetto è raggiungibile.
    """
    if robot_pos is None or object_pos is None:
        return None, False

    # Distanza sul piano x-y
    distance = math.sqrt(
        (robot_pos[0] - object_pos[0]) ** 2 +
        (robot_pos[1] - object_pos[1]) ** 2 #+
        #(robot_pos[2] - object_pos[2]) ** 2
    )
    return distance, distance <= max_reach

def get_current_scene_graph_json(sim, identified_objects):
    object_positions, robot_position = get_object_positions_from_sim(sim, identified_objects, 'Franka')

    logger.debug("\n--- FASE 3: Analisi e Creazione dello Scene Graph Arricchito ---")
    scene_graph = []

    for name, position in object_positions.items():
        distance, reachable = analyze_reachability(robot_position, position)

        scene_graph.append({
            "nome_oggetto": name,
            "posizione_3D": [round(p, 3) for p in position],
            "distanza_dal_robot_m": round(distance, 3) if distance is not None else "N/A",
            "raggiungibile": reachable
        })

    scene_graph_json = json.dumps(scene_graph, indent=2)
    # Stampa finale dello Scene Graph
    logger.debug(f"""
    SCENE GRAPH ARRICCHITO FINALE
    -----------------------------------
    {scene_graph_json}
    -----------------------------------
    """)
    return scene_graph_json

def get_current_scene_graph_data_from_objects_list(sim, robot_name, objects_handle):
    logger.debug("Costruzione dello Scene Graph completo (Manipolabili + Location)...")
    # 1. Inizializza il dizionario con le due liste vuote, come hai chiesto
    scene_data = {
        "manipulable_objects": [],
        "static_locations": []
    }

    full_robot_name = robot_name if robot_name.startswith('/') else f'/{robot_name}'
    robot_handle = sim.getObject(f'{full_robot_name}')

    # Liste temporanee di handle per i controlli
    manipulable_handles_map = {}  # Dizionario {handle: nome}
    static_handles_map = {}  # Dizionario {handle: nome}
    all_objects_handles_map = {}
    for object_handle in objects_handle:
        name = sim.getObjectAlias(object_handle) or sim.getObjectName(object_handle)

        is_dynamic = sim.getBoolProperty(object_handle, 'dynamic')
        is_respondable = sim.getBoolProperty(object_handle, 'respondable')
        logger.debug(f"{name} is DYNAMIC={is_dynamic} and RESPONDABLE={is_respondable}")

        all_objects_handles_map[object_handle] = name
        if is_dynamic and is_respondable:
            # È un oggetto che possiamo afferrare (es. Cuboid, martello)
            manipulable_handles_map[object_handle] = name
        else:
            # È un oggetto statico ma "solido" (es. Pad, Tavolo)
            static_handles_map[object_handle] = name

    # Popola le location statiche
    for object_handle, name in static_handles_map.items():
        dist_from_robot = dist(sim, object_handle, robot_handle)
        is_reachable = dist_from_robot <= 0.855

        object_info = {
            "nome": name,
            # "posizione_3D": [round(p, 3) for p in position],
            "distanza_dal_robot_m": round(dist_from_robot, 3) if dist_from_robot is not None else "N/A",
            "raggiungibile": is_reachable
        }
        scene_data["static_locations"].append(object_info)

    # Popola gli oggetti manipolabili (con la nuova logica)
    for object_handle, name in manipulable_handles_map.items():
        dist_from_robot = dist(sim, object_handle, robot_handle)
        is_reachable = dist_from_robot <= 0.855

        # --- NUOVA LOGICA "POGGIATO_SU" ---
        poggiato_su = "sconosciuto"  # Default
        min_dist_loc = float('inf')
        for loc_handle, loc_name in all_objects_handles_map.items():
            if object_handle==loc_handle:
                continue

            if is_on(sim, object_handle, loc_handle):
                poggiato_su = loc_name
        # ------------------------------------

        object_info = {
            "nome": name,
            # "posizione_3D": [round(p, 3) for p in position],
            "distanza_dal_robot_m": round(dist_from_robot, 3) if dist_from_robot is not None else "N/A",
            "poggiato_su": poggiato_su,
            "raggiungibile": is_reachable
        }

        scene_data["manipulable_objects"].append(object_info)

    return scene_data

def is_on(sim, object_handle, target_handle):
    _, _, object_dimensions = sim.getShapeGeomInfo(object_handle)
    _, _, target_dimensions = sim.getShapeGeomInfo(target_handle)
    object_position = sim.getObjectPosition(object_handle, -1)
    object_z = object_position[2]
    target_position = sim.getObjectPosition(target_handle, -1)
    target_z = target_position[2]

    distance = dist(sim, object_handle, target_handle)
    if dist_xy(sim, object_handle, target_handle) < target_dimensions[0] / 2 and object_z > target_z:
        return True
    return False


def get_current_scene_graph_data_from_objects_list_OLD(sim, robot_name, objects_handle):
    logger.debug("Costruzione dello Scene Graph completo (Manipolabili + Location)...")
    # 1. Inizializza il dizionario con le due liste vuote, come hai chiesto
    scene_data = {
        "manipulable_objects": [],
        "static_locations": []
    }

    full_robot_name = robot_name if robot_name.startswith('/') else f'/{robot_name}'
    robot_handle = sim.getObject(f'{full_robot_name}')

    for object_handle in objects_handle:
        name = sim.getObjectAlias(object_handle) or sim.getObjectName(object_handle)

        is_dynamic = sim.getBoolProperty(object_handle, 'dynamic')
        is_respondable = sim.getBoolProperty(object_handle, 'respondable')
        # print(f"\nAnalizzo oggetto: '{name}' (Handle: {object_handle})")
        # print(f"  - È Dynamic?   -> {is_dynamic}")
        # print(f"  - È Respondable? -> {is_respondable}")

        dist_from_robot = dist(sim, object_handle, robot_handle)
        is_reachable = dist_from_robot <= 0.855

        object_info = {
            "nome": name,
            # "posizione_3D": [round(p, 3) for p in position],
            "distanza_dal_robot_m": round(dist_from_robot, 3) if dist_from_robot is not None else "N/A",
            "raggiungibile": is_reachable
        }

        # --- 2. CLASSIFICAZIONE E INSERIMENTO NELLE LISTE ---
        if is_dynamic and is_respondable:
            # È un oggetto che possiamo afferrare (es. Cuboid, martello)
            scene_data["manipulable_objects"].append(object_info)
        else:
            # È un oggetto statico ma "solido" (es. Pad, Tavolo)
            scene_data["static_locations"].append(object_info)

    return scene_data

def convert_scene_graph_data_to_json(scene_data):
    scene_graph_json = json.dumps(scene_data, indent=2)
    # Stampa finale dello Scene Graph
    logger.debug(f"""
        SCENE GRAPH ARRICCHITO FINALE
        -----------------------------------
        {scene_graph_json}
        -----------------------------------
        """)
    return scene_graph_json

def get_scene_graph_json(sim, robot_name, objects_handle):
    scene_data = get_current_scene_graph_data_from_objects_list(sim, robot_name, objects_handle)
    return convert_scene_graph_data_to_json(scene_data)

def get_robot_holding_state(sim, robot_handles, object_handles):
    """
    Controlla se il robot sta tenendo uno degli oggetti manipolabili.

    Args:
        sim: Oggetto client API 'sim'.
        robot_handles: Dizionario degli handle (serve 'tip').
        object_handles (list): Lista degli handle degli oggetti che POSSONO essere presi.

    Returns:
        str: Il nome dell'oggetto tenuto, o "libero" se la pinza è vuota.
    """
    gripper_tip_handle = robot_handles['tip']
    holding_state = "libero"

    for handle in object_handles:
        try:
            # Chiedi chi è il genitore di questo oggetto (OPPURE: check distanza. Alla fine forse useremo VISIONE)
            parent_handle = sim.getObjectParent(handle)

            # Se il genitore è la punta della pinza, il robot lo sta tenendo
            if parent_handle == gripper_tip_handle:
                name = sim.getObjectAlias(handle) or sim.getObjectName(handle)
                holding_state = name  # Restituisce il nome dell'oggetto tenuto
                break
        except Exception:
            continue  # Ignora errori per questo oggetto

    logger.debug(f"✅ World State creato. Stato pinza: {holding_state}")
    return holding_state # Se il loop finisce, la pinza è vuota

def get_world_state_data(sim, robot_object, objects_handles):
    """
    Costruisce un dizionario completo dello stato del mondo, includendo
    lo stato del robot (cosa tiene) e lo stato dell'ambiente.
    """
    logger.debug("Costruzione del World State completo...")

    # 1. Inizializza la struttura dati
    world_state = {
        "robot_state": {
            "is_holding": "libero"  # Default
        },
        "environment_objects": {
            "manipulable_objects": [],
            "static_locations": []
        }
    }

    # 2. Controlla lo stato della pinza
    # Ora che abbiamo la lista degli oggetti manipolabili, controlliamo se ne stiamo tenendo uno
    stato_presa = get_robot_holding_state(sim, robot_object.handles, objects_handles)
    world_state["robot_state"]["is_holding"] = stato_presa

    world_state["environment_objects"] = get_current_scene_graph_data_from_objects_list(sim, "Franka", objects_handles)

    return world_state

def get_world_state_json(sim, robot_object, objects_handles):
    world_state_dict = get_world_state_data(sim, robot_object, objects_handles)
    return json.dumps(world_state_dict, indent=2)

def dist_xy(sim, target_handle, ref_handle):
    target_pos = sim.getObjectPosition(target_handle)
    ref_pos = sim.getObjectPosition(ref_handle)
    xy_dist = math.sqrt(
        (target_pos[0] - ref_pos[0]) ** 2 +
        (target_pos[1] - ref_pos[1]) ** 2
    )
    return xy_dist

def dist(sim, target_handle, ref_handle):
    target_pos = sim.getObjectPosition(target_handle)
    ref_pos = sim.getObjectPosition(ref_handle)
    distance = math.sqrt(
        (target_pos[0] - ref_pos[0]) ** 2 +
        (target_pos[1] - ref_pos[1]) ** 2 +
        (target_pos[2] - ref_pos[2]) ** 2
    )
    return distance

def convert_object_names_to_handles(sim, object_names):
    object_handles = []
    for name in object_names:
        full_name = name if name.startswith('/') else f'/{name}'
        object_handles.append(sim.getObject(full_name))
    return object_handles

def simulation_stepper(client, stop_event, step_size_ms=50):
    """
    Funzione da eseguire in un thread separato.
    Chiama client.step() a intervalli regolari finché 'stop_event' non viene impostato.
    """
    try:
        while not stop_event.is_set():
            client.step()  # Esegui un passo di simulazione
            # Pausa per 'step_size_ms' millisecondi
            time.sleep(step_size_ms / 1000.0)
    except Exception as e:
        # Gestisce l'errore se la connessione si interrompe
        logger.exception(f"Errore nel thread del simulatore: ")