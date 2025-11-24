import logging
import time
import math
from PIL import Image
import numpy as np

logger = logging.getLogger(__name__)

ROBOT_FINGER_PAD_HEIGHT = 0.015
MAX_FRANKA_REACH = 0.855

class CoppeliaPerception:
    def __init__(self, sim, handles):
        self.sim = sim
        self.handles = handles
        self.vision_sensor_handle = sim.getObject('/visionSensor')
        # Counter delle immagini scattate
        self.no_images = 0
        # STATO INTERNO
        # Dizionario {nome_oggetto: handle} di TUTTO ciò che esiste nella scena
        self.ground_truth_objects = self._scan_groud_truth_objects(['Floor', 'Franka', 'FakeFranka', 'Blue_Wall'])
        # Lista [nomi_oggetti] di ciò che è stato RILEVATO nell'ultimo scan
        self.identified_objects_names = []

    def _scan_groud_truth_objects(self, ignore_list=None):
        """

        Interroga la scena di CoppeliaSim per identificare tutti gli oggetti
        di primo livello (figli diretti della scena).

        Questo metodo utilizza l'approccio corretto e più efficiente, sfruttando
        il parametro 'options' della funzione sim.getObjectsInTree per delegare
        il filtraggio gerarchico al motore di simulazione.

        :return: Una lista di stringhe contenente i nomi degli oggetti che
                 soddisfano tutti i criteri.
        """
        logger.debug("Recupero degli oggetti interattivi di primo livello (Metodo Ottimizzato)...")
        sim = self.sim
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

        ground_truth_objects = {}
        for handle in all_top_level_objects:
            try:
                # Recupera il nome più user-friendly: l'alias se esiste, altrimenti il nome dell'oggetto.
                # Questa è una best practice per l'identificazione degli oggetti.[4]
                name = sim.getObjectAlias(handle)
                if name in ignore_list:
                    continue
                ground_truth_objects[name] = handle

            except Exception:
                logger.exception(f"\n  --> ⚠️ ERRORE o oggetto non pertinente {handle}: ")

        logger.info(
            f"Ground Truth aggiornata: {len(ground_truth_objects)} oggetti noti ({list(ground_truth_objects.keys())})")
        return ground_truth_objects

    def perceive_scene(self, gemini_client=None, use_vision=True):
        """
        Esegue il ciclo di percezione completo:
        1. Scatta foto.
        2. (Opzionale) Chiede a Gemini cosa vede.
        3. (Fallback) Usa la Ground Truth se la visione è disabilitata o fallisce.

        Popola self.identified_objects_names.
        """
        # 1. Acquisisci immagine (sempre utile per il log o per Gemini)
        image = self.get_camera_image()

        if use_vision and gemini_client:
            logger.info("Avvio percezione visiva con Gemini...")
            # Passiamo a Gemini la lista dei nomi noti per aiutare il riconoscimento (Closed-Set Recognition)
            known_names = list(self.ground_truth_objects.keys())

            # Chiamata a Gemini (assumendo che GeminiClient abbia questo metodo)
            detected_names = gemini_client.get_object_list_from_image(image, known_names)

            if detected_names:
                # Filtra: Accetta solo nomi che esistono davvero nella Ground Truth (Anti-Allucinazione)
                valid_detections = [name for name in detected_names if name in self.ground_truth_objects]
                self.identified_objects_names = valid_detections
                logger.info(f"Gemini ha visto: {self.identified_objects_names}")
            else:
                logger.warning("Gemini non ha rilevato oggetti. Uso Ground Truth come fallback.")
                self.identified_objects_names = list(self.ground_truth_objects.keys())

        else:
            # Modalità "Cieca" o "Debug": Il robot "vede" tutto ciò che esiste (Oracle)
            logger.debug("Percezione visiva saltata. Uso Ground Truth (Oracle Mode).")
            self.identified_objects_names = list(self.ground_truth_objects.keys())

        return self.identified_objects_names

    def _scan_scene_and_robot_state(self, objects_handles):
        """
        Costruisce un dizionario completo dello stato del mondo, includendo
        lo stato del robot (cosa tiene) e lo stato dell'ambiente.
        """
        robot_base = self.handles['base']
        gripper_tip = self.handles['tip']

        # Struttura dati finale
        world_state = {
            "robot_state": {"is_holding": "libero"},
            "environment_objects": {
                "manipulable_objects": [],
                "static_locations": []
            }
        }

        # Cache locale per evitare ri-calcoli
        # Format: {handle: {'name': str, 'pos': [x,y,z], 'is_dynamic': bool}}
        objects_cache = {}

        # 1. PRIMO PASSAGGIO: Raccolta dati e check "is_holding"
        # Identifichiamo subito se stiamo tenendo qualcosa per escluderlo dalla logica "poggiato su"
        held_object_handle = None

        for object_handle in objects_handles:
            # Ottieni alias
            name = self.sim.getObjectAlias(object_handle)

            # Ottieni proprietà fisiche
            is_dynamic = self.sim.getBoolProperty(object_handle, 'dynamic')
            is_respondable = self.sim.getBoolProperty(object_handle, 'respondable')

            # Ottieni posizione assoluta per calcoli veloci
            pos = self.sim.getObjectPosition(object_handle, -1)

            # Check se è tenuto dal robot (gerarchia)
            parent = self.sim.getObjectParent(object_handle)
            if parent == gripper_tip:
                world_state["robot_state"]["is_holding"] = name
                # Non aggiungiamo l'oggetto alla cache per i calcoli spaziali ambientali
                continue

            objects_cache[object_handle] = {
                'name': name,
                'pos': pos,
                'is_manipulable': is_dynamic and is_respondable  # o la tua logica combinata
            }

        # 2. SECONDO PASSAGGIO: Costruzione Scene Graph e Relazioni
        for object_handle, data in objects_cache.items():
            # Calcola distanza dal robot (usando la posizione relativa alla base per precisione o assoluta per stima)
            dist_from_robot = self.dist(object_handle, robot_base)
            is_reachable = dist_from_robot <= MAX_FRANKA_REACH

            obj_info = {
                "nome": data['name'],
                # "posizione_3D": [round(p, 3) for p in position],
                "distanza_dal_robot_m": round(dist_from_robot, 3) if dist_from_robot is not None else "N/A",
                "raggiungibile": is_reachable
            }

            if data['is_manipulable']:
                # Calcola "poggiato_su" solo per oggetti manipolabili
                poggiato_su = self._poggiato_su_GEMINI(object_handle, objects_cache)

                obj_info["poggiato_su"] = poggiato_su
                world_state["environment_objects"]["manipulable_objects"].append(obj_info)
            else:
                # Location statica
                world_state["environment_objects"]["static_locations"].append(obj_info)

        return world_state

    def get_world_state_data(self):
        logger.debug("Costruzione del World State...")
        identified_objects_handles = [self.ground_truth_objects.get(name) for name in self.identified_objects_names]
        return self._scan_scene_and_robot_state(identified_objects_handles)

    def get_camera_image(self):
        try:
            img, res = self.sim.getVisionSensorImg(self.vision_sensor_handle)
            img_np = np.frombuffer(img, dtype=np.uint8).reshape((res[1], res[0], 3))
            img_np = np.flipud(img_np)
            pil_image = Image.fromarray(img_np, 'RGB')
            pil_image.save(f"scatto_simulatore_{self.no_images}.png")
            self.no_images += 1
        except Exception as e:
            logger.exception(f"❌ Errore nell'acquisizione dell'immagine: {e}")
            return None
        return pil_image

    def get_grasp_pose(self, object_name):
        """
        Calcola la posa di presa usando il Bounding Box (BB) per maggiore precisione.
        """
        obj_handle = self.sim.getObject(f'/{object_name}')
        if not obj_handle: return None

        try:
            # 1. Ottieni la posa base dell'oggetto (il suo pivot) rispetto alla base del robot
            # Matrice 4x4 (appiattita a 12 elementi)
            object_matrix = self.sim.getObjectMatrix(obj_handle, self.handles['base'])

            # 2. Ottieni il Bounding Box (BB)
            # size: [x_size, y_size, z_size]
            # rel_pose: [x, y, z, qx, qy, qz, qw] posizione del centro del BB relativa al pivot dell'oggetto
            size, rel_pose = self.sim.getShapeBB(obj_handle)

            object_height = size[2]  # La dimensione Z è l'altezza

            # 3. Calcola il VERO CENTRO geometrico
            # Spesso il pivot di un oggetto è alla base, ma noi vogliamo prendere il centro.
            # getShapeBB ci dice dov'è il centro rispetto al pivot.

            # Se l'oggetto è semplice, possiamo approssimare sommando l'offset Z relativo
            # alla posizione Z della matrice dell'oggetto.
            # Nota: Per una precisione assoluta servirebbe moltiplicare le matrici,
            # ma per cubi e cilindri non ruotati, sommare l'offset locale alla Z globale è sufficiente.

            center_z_offset = rel_pose[2]

            # --- DEFINIZIONE POSE ---

            # GRASP: Vogliamo che la pinza vada al centro geometrico dell'oggetto?
            # O leggermente sopra?
            grasp_pose = list(object_matrix)

            # Correzione altezza:
            # Partiamo dalla Z del pivot (grasp_pose[11])
            # Aggiungiamo l'offset per arrivare al centro geometrico (center_z_offset)
            # Se vuoi prendere l'oggetto "dall'alto", potresti voler aggiungere ancora qualcosa (es. size[2]/2)
            # Per ora, puntiamo al centro geometrico esatto:
            grasp_pose[11] += center_z_offset

            # Afferriamo l'oggetto da "sopra"
            grasp_pose[11] += object_height/2.0 - ROBOT_FINGER_PAD_HEIGHT

            return grasp_pose

        except Exception as e:
            logger.error(f"Errore calcolo pose per '{object_name}': {e}")
            return None

    def get_place_pose(self, object_name, target_name):
        """
        Calcola la posa esatta di rilascio (Place) affinché l'oggetto
        sia appoggiato sopra il target (stacking).

        La posa restituita corrisponde alla posizione che deve assumere il
        Gripper Tip (assumendo che abbia afferrato l'oggetto al centro).
        """
        obj_handle = self.sim.getObject(f'/{object_name}')
        target_handle = self.sim.getObject(f'/{target_name}')

        if not obj_handle or not target_handle:
            return None

        try:
            # 1. Ottieni la posa del TARGET (es. il Pad) rispetto alla base del robot
            # Questa matrice definisce X, Y e l'orientamento di base
            target_matrix = self.sim.getObjectMatrix(target_handle, self.handles['base'])

            # 2. Ottieni le dimensioni precise (Bounding Box)
            # size: [x, y, z], pose: [x, y, z, qx, qy, qz, qw] (relativa al pivot)
            size_obj, pose_obj = self.sim.getShapeBB(obj_handle)
            size_tgt, pose_tgt = self.sim.getShapeBB(target_handle)

            obj_height = size_obj[2]
            tgt_height = size_tgt[2]

            # 3. Calcolo della Z di "Stacking" (Impilamento)
            # Vogliamo che: Fondo_Oggetto tocchi Cima_Target
            logger.debug(f"target_matrix_z: {target_matrix[11]}")
            logger.debug(f"pose_tgt: {pose_tgt[2]}")
            logger.debug(f"target_height: {tgt_height}")
            # A. Trova la Z della superficie superiore del Target
            # Z_pivot_target + Offset_Centro_BB + Metà_Altezza
            target_surface_z = target_matrix[11] + pose_tgt[2] + (tgt_height / 2.0)
            logger.debug(f"target_surface_z: {target_surface_z}")

            # B. Trova dove deve stare il centro del Gripper/Oggetto
            # Superficie_Target + Metà_Altezza_Oggetto
            # (Assumiamo che la presa sia avvenuta al centro geometrico dell'oggetto)
            final_z = target_surface_z + (obj_height / 2.0)

            # 4. Costruisci la matrice di Place
            place_pose = list(target_matrix)
            place_pose[11] = final_z + obj_height/2.0 - ROBOT_FINGER_PAD_HEIGHT # (TODO) Sincronizziamo con il punto di afferraggio

            time.sleep(4.0)

            # Nota: Questo posizionerà l'oggetto al centro X,Y del target,
            # con lo stesso orientamento del target.

            # 5. Restituisci anche l'altezza dell'oggetto, utile per calcolare
            # le distanze di sicurezza (pre-place, lift) nel chiamante.
            return place_pose, obj_height

        except Exception as e:
            logger.error(f"Errore calcolo place pose per '{object_name}' su '{target_name}': {e}")
            return None, 0.0

    def dist_xy(self, target_handle, ref_handle):
        target_pos = self.sim.getObjectPosition(target_handle)
        ref_pos = self.sim.getObjectPosition(ref_handle)
        xy_dist = math.sqrt(
            (target_pos[0] - ref_pos[0]) ** 2 +
            (target_pos[1] - ref_pos[1]) ** 2
        )
        return xy_dist

    def dist(self, target_handle, ref_handle):
        target_pos = self.sim.getObjectPosition(target_handle)
        ref_pos = self.sim.getObjectPosition(ref_handle)
        distance = math.sqrt(
            (target_pos[0] - ref_pos[0]) ** 2 +
            (target_pos[1] - ref_pos[1]) ** 2 +
            (target_pos[2] - ref_pos[2]) ** 2
        )
        return distance

    def is_on(self, object_handle, target_handle):
        sim = self.sim
        _, _, object_dimensions = sim.getShapeGeomInfo(object_handle)
        _, _, target_dimensions = sim.getShapeGeomInfo(target_handle)
        object_position = sim.getObjectPosition(object_handle, -1)
        object_z = object_position[2]
        target_position = sim.getObjectPosition(target_handle, -1)
        target_z = target_position[2]

        # distance = dist(sim, object_handle, target_handle)
        if self.dist_xy(object_handle, target_handle) < target_dimensions[0] / 2 and object_z > target_z:
            return True
        return False

    def _poggiato_su_GEMINI(self, object_handle, objects_cache):
        # Ottimizzazione: Cerca solo tra gli ALTRI oggetti nella cache che sono SOTTO questo
        data = objects_cache[object_handle]
        poggiato_su = "sconosciuto"
        for potential_support_handle, support_data in objects_cache.items():
            if object_handle == potential_support_handle: continue

            obj_z = data['pos'][2]
            supp_z = support_data['pos'][2]

            # Definisci una soglia verticale e orizzontale
            # Check veloce Z: l'oggetto deve essere sopra il supporto (es. entro 5cm)
            if 0 < (obj_z - supp_z) < 0.10:
                # Check distanza XY (sono sovrapposti?)
                # Calcolo distanza planare semplice
                dist_xy = ((data['pos'][0] - support_data['pos'][0]) ** 2 +
                           (data['pos'][1] - support_data['pos'][1]) ** 2) ** 0.5

                # Se sono vicini in XY (es. < 10cm), assumiamo contatto
                # (Per una logica più precisa serve Bounding Box check)
                if dist_xy < 0.15:
                    poggiato_su = support_data['name']
                    break  # Trovato un supporto, fermati

        return poggiato_su

    def pick_and_hold_success_detector(self, object_name):
        object_handle = self.sim.getObject(f'/{object_name}')

        if self.dist(object_handle, self.handles['tip']) > 0.05:
            return False, "Oggetto non afferrato"
        return True, "True"

    def place_success_detector(self, object_name, target_name):
        object_handle = self.sim.getObject(f'/{object_name}')
        target_handle = self.sim.getObject(f'/{target_name}')

        if self.is_on(object_handle, target_handle):
            return True, "True"
        return False, "Oggetto non lasciato su target"