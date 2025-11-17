import google.generativeai as genai
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np

import json
import math

from tesi_gemini_robotics.utils import get_image_of_scene

# --- 1. CONFIGURAZIONE ---
GEMINI_API_KEY = os.getenv("GOOGLE_API_KEY")
try:
    genai.configure(api_key=GEMINI_API_KEY)
except Exception as e:
    print(f"❌ Errore nella configurazione di Gemini. Verifica la tua chiave API. Dettagli: {e}")
    exit()

# Scegli il modello da usare. Quello per la robotica è ideale.
MODEL_NAME = 'models/gemini-robotics-er-1.5-preview'
#MODEL_NAME = 'models/gemma-3-12b-it'

# Parametri del robot (da adattare al tuo modello in CoppeliaSim)
ROBOT_BASE_NAME = '/Franka'  # Il nome dell'oggetto base del tuo robot
MAX_REACH_METERS = 0.855  # Raggio d'azione approssimativo del braccio in metri


# --- 2. FUNZIONI DI INTERAZIONE ---

def get_top_level_interactive_objects(sim, ignore_list=['Floor']):
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
    print("Recupero degli oggetti interattivi di primo livello (Metodo Ottimizzato)...")
    object_names = []

    # --- LA LOGICA CORRETTA BASATA SULLA RICERCA DOCUMENTALE ---

    # 1. Ottiene gli handle di TUTTI gli oggetti di primo livello nella scena.
    #    - treeBaseHandle = sim.handle_scene: La ricerca parte dalla radice della scena.
    #    - objectType = sim.handle_all: La ricerca è generalizzata a qualsiasi
    #      tipo di oggetto per massima robustezza (non solo 'shape').
    #    - options = 2: QUESTA È LA CHIAVE. Il valore 2 (bit 1 impostato)
    #      istruisce la funzione a restituire SOLO i figli diretti della radice,
    #      escludendo tutti i sotto-oggetti in gerarchie più profonde.
    all_top_level_objects = sim.getObjectsInTree(sim.handle_scene, sim.sceneobject_shape, 2) #sim.handle_all
    print(f"Trovati {len(all_top_level_objects)} oggetti totali di primo livello nella scena.")

    # ----------------------------------------------------------------

    for handle in all_top_level_objects:
        name = ""  # Inizializza il nome per l'uso nel blocco except
        try:
            # Recupera il nome più user-friendly: l'alias se esiste, altrimenti il nome dell'oggetto.
            # Questa è una best practice per l'identificazione degli oggetti.[4]
            name = sim.getObjectAlias(handle) or sim.getObjectName(handle)
            if name in ignore_list:
                continue
                
            # Il filtraggio delle proprietà rimane identico all'approccio iniziale,
            # poiché era già corretto.
            is_dynamic = sim.getBoolProperty(handle, 'dynamic')
            is_respondable = sim.getBoolProperty(handle, 'respondable')

            print(f"\nAnalizzo oggetto: '{name}' (Handle: {handle})")
            print(f"  - È Dynamic?   -> {is_dynamic}")
            print(f"  - È Respondable? -> {is_respondable}")

            if is_dynamic and is_respondable:
                print(f"  --> ✅ CONDIZIONI SODDISFATTE. Aggiungo '{name}' alla lista.")
                object_names.append(name)
            else:
                print(f"  --> ❌ CONDIZIONI NON SODDISFATTE. Ignoro l'oggetto.")

        except Exception as e:
            # Un blocco try-except è fondamentale qui. Poiché cerchiamo oggetti
            # di tipo 'sim.handle_all', potremmo incontrare oggetti (es. script,
            # collezioni) che non supportano le proprietà 'dynamic' o 'respondable'.
            # Questo blocco previene il crash dello script e gestisce l'errore.
            object_identity = f"'{name}'" if name else f"con handle {handle}"
            print(f"\n  --> ⚠️ ERRORE o oggetto non pertinente {object_identity}: {e}")

    print(f"\n✅ Trovati {len(object_names)} oggetti manipolabili di primo livello: {object_names}")
    return object_names


def get_object_list_from_gemini(image, available_objects):
    """
    Invia l'immagine e la lista di oggetti a Gemini per l'identificazione.
    """
    print("\n--- FASE 1: Percezione Vincolata (Gemini) ---")
    model = genai.GenerativeModel(MODEL_NAME)

    # Formatta la lista di oggetti per il prompt
    object_list_str = ", ".join(available_objects)

    prompt = f"""
    Sei un sistema di percezione per un robot. Il tuo compito è identificare quali oggetti, da una lista predefinita, sono presenti nell'immagine.

    Lista di oggetti possibili nella scena: [{object_list_str}]

    Analizza l'immagine e restituisci una risposta ESCLUSIVAMENTE in formato JSON, con una singola chiave "oggetti_visti" contenente una lista di stringhe con i nomi degli oggetti che hai effettivamente visto.
    Usa SOLO E SOLTANTO i nomi esatti forniti nella lista qui sopra.

    ESEMPIO:
    Input: [Immagine con un martello e una vite]
    Output JSON atteso:
    {{"oggetti_visti": ["martello", "vite_metallo"]}}
    """

    contenuto_richiesta = [image, prompt]

    try:
        print("Richiesta di identificazione oggetti a Gemini...")
        response = model.generate_content(contenuto_richiesta)
        #print("RISPOSTA COMPLETA") #DEBUG
        #print(response.text)       #DEBUG
        testo_risposta = response.text.strip().replace("```json", "").replace("```", "")
        dati_json = json.loads(testo_risposta)
        print(testo_risposta)

        object_names = dati_json.get("oggetti_visti", [])
        print(f"✅ Gemini ha identificato: {object_names}")
        return object_names
    except Exception as e:
        print(f"❌ Errore durante l'analisi dell'immagine con Gemini: {e}")
        return []


def get_object_positions_from_sim(sim, object_names):
    """
    FASE 2: Prende la lista di nomi e interroga CoppeliaSim per le coordinate 3D.
    Restituisce un dizionario con le posizioni e la posizione del robot.
    """
    print("\n--- FASE 2: Arricchimento Dati (CoppeliaSim) ---")
    object_positions = {}

    for name in object_names:
        try:
            # Aggiungiamo il '/' se non è presente per la ricerca nella gerarchia
            full_name = name if name.startswith('/') else f'/{name}'
            handle = sim.getObject(full_name)

            # -1 indica che vogliamo le coordinate rispetto al frame del mondo
            position = sim.getObjectPosition(handle, -1)
            object_positions[name] = position
            print(f"✅ Posizione di '{name}': {np.round(position, 2)}")
        except Exception:
            print(f"⚠️ Attenzione: Gemini ha menzionato '{name}', ma non è stato trovato nella scena di CoppeliaSim.")

    # Ottieni anche la posizione della base del robot
    try:
        robot_handle = sim.getObject(ROBOT_BASE_NAME)
        robot_position = sim.getObjectPosition(robot_handle, -1)
        print(f"✅ Posizione del robot '{ROBOT_BASE_NAME}': {np.round(robot_position, 2)}")
    except Exception:
        print(f"❌ Errore: l'oggetto base del robot '{ROBOT_BASE_NAME}' non è stato trovato. Controlla il nome.")
        robot_position = None

    return object_positions, robot_position


def analyze_reachability(robot_pos, object_pos, max_reach):
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


# --- 3. PROGRAMMA PRINCIPALE ---

def main():
    # Connessione a CoppeliaSim
    try:
        client = RemoteAPIClient()
        sim = client.require('sim')
    except Exception as e:
        print(f"❌ Errore di connessione a CoppeliaSim: {e}")
        return

    # Ottieni la lista di nomi dal simulatore PRIMA di tutto
    objects_to_ignore = ['Floor', 'Franka', 'FakeFranka']
    available_objects = get_top_level_interactive_objects(sim, ignore_list=objects_to_ignore)

    # Acquisizione immagine
    pil_image = get_image_of_scene(sim)

    # --- ESECUZIONE DEL FLUSSO IBRIDO ---

    # 1. Gemini identifica gli oggetti
    # Passa la lista a Gemini
    identified_names = get_object_list_from_gemini(pil_image, available_objects)
    if not identified_names:
        print("Nessun oggetto identificato. Termino il programma.")
        return

    # 2. CoppeliaSim fornisce le coordinate
    # Ora usa la lista 'identified_names' (è garantita essere corretta)
    # per la fase di arricchimento dati e la creazione dello scene graph.
    object_positions, robot_position = get_object_positions_from_sim(sim, identified_names)

    # 3. Costruzione e analisi dello Scene Graph Arricchito
    print("\n--- FASE 3: Analisi e Creazione dello Scene Graph Arricchito ---")
    scene_graph = []

    for name, position in object_positions.items():
        distance, reachable = analyze_reachability(robot_position, position, MAX_REACH_METERS)

        scene_graph.append({
            "nome_oggetto": name,
            "posizione_3D": [round(p, 3) for p in position],
            "distanza_dal_robot_m": round(distance, 3) if distance is not None else "N/A",
            "raggiungibile": reachable
        })

    # Stampa finale dello Scene Graph
    print("\n" + "=" * 50)
    print("         SCENE GRAPH ARRICCHITO FINALE")
    print("=" * 50)
    print(json.dumps(scene_graph, indent=2))
    print("=" * 50)


if __name__ == '__main__':
    main()