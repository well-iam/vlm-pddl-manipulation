import google.generativeai as genai
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
from PIL import Image
import os

# --- FASE 1: CONFIGURAZIONE DI GEMINI ---
# Incolla qui la tua chiave API.
# (Per maggiore sicurezza, è meglio usare le variabili d'ambiente, ma per iniziare va bene così)
GEMINI_API_KEY = os.getenv("GOOGLE_API_KEY")
genai.configure(api_key=GEMINI_API_KEY)

# Inizializza il modello multimodale (visione + testo)
model = genai.GenerativeModel('gemini-robotics-er-1.5-preview')
print("✅ Modello Gemini inizializzato.")

# --- FASE 2: CONNESSIONE E CONTROLLO DI COPPELIASIM ---
print("Tentativo di connessione a CoppeliaSim...")
client = RemoteAPIClient()
sim = client.require('sim')
print("✅ Connesso a CoppeliaSim.")

# Ottieni un "handle" per la telecamera nella scena.
# NOTA: Assicurati che nella tua scena di CoppeliaSim ci sia un sensore di visione
# chiamato 'Vision_sensor'. Se ha un altro nome, modificalo qui.
try:
    vision_sensor_handle = sim.getObject('/visionSensor')
    print(f"✅ Trovato sensore di visione con handle: {vision_sensor_handle}")
except Exception as e:
    print(f"❌ Errore: Assicurati che ci sia un oggetto chiamato '/Vision_sensor' nella tua scena di CoppeliaSim.")
    exit()

# --- FASE 3: ACQUISIZIONE E CONVERSIONE DELL'IMMAGINE ---
print("Scattando una foto dal simulatore...")
# Ottieni l'immagine grezza e la sua risoluzione
img, res = sim.getVisionSensorImg(vision_sensor_handle)

if img:
    # L'immagine è una stringa di byte. Dobbiamo convertirla in un formato corretto.
    # 1. Converti la stringa di byte in un array NumPy
    img_np = np.frombuffer(img, dtype=np.uint8)

    # 2. Riorganizza l'array nella forma corretta (Altezza, Larghezza, Canali)
    img_np = img_np.reshape((res[1], res[0], 3))

    # 3. CoppeliaSim restituisce l'immagine capovolta. Raddrizziamola.
    img_np = np.flipud(img_np)

    # 4. Converti l'array NumPy in un oggetto Immagine (formato richiesto da Gemini)
    pil_image = Image.fromarray(img_np, 'RGB')

    # (Opzionale) Salva l'immagine su disco per verificare che sia corretta
    pil_image.save("scatto_simulatore.png")
    print("✅ Immagine scattata e salvata come 'scatto_simulatore.png'.")

    # --- FASE 4: QUERY MULTIMODALE A GEMINI ---
    prompt_testo = "Descrivi questa scena. Quali oggetti vedi e di che colore sono? Quale azione potrei compiere?"

    print(f"\nInviando la query a Gemini con il prompt: '{prompt_testo}'")

    # Crea il contenuto della richiesta: una lista con testo e immagine
    contenuto_richiesta = [prompt_testo, pil_image]

    # Invia la richiesta al modello
    response = model.generate_content(contenuto_richiesta)

    # --- FASE 5: VISUALIZZAZIONE DELLA RISPOSTA ---
    print("\n--- RISPOSTA DI GEMINI ---")
    print(response.text)
    print("--------------------------\n")

else:
    print("❌ Fallimento: non è stato possibile catturare l'immagine dal sensore di visione.")

print("Programma terminato.")