import google.generativeai as genai
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
from PIL import Image
import os
import json

# --- 1. CONFIGURAZIONE INIZIALE ---
# Incolla qui la tua chiave API di Gemini.
GEMINI_API_KEY = os.getenv("GOOGLE_API_KEY")
try:
    genai.configure(api_key=GEMINI_API_KEY)
except Exception as e:
    print(f"❌ Errore nella configurazione di Gemini. Verifica la tua chiave API. Dettagli: {e}")
    exit()

# Nomi dei modelli disponibili
MODEL_ROBOTICS = 'models/gemini-robotics-er-1.5-preview'
MODEL_GEMMA = 'models/gemma-3-27b-it'

# PROMPT TEMPLATE
master_prompt_template = """
Sei un pianificatore robotico. Il tuo compito è osservare una scena e un'istruzione di alto livello, 
e generare la prossima, singola azione da compiere.

Rispondi SEMPRE E SOLO con un oggetto JSON valido, senza testo aggiuntivo prima o dopo.

Il formato JSON deve contenere le seguenti chiavi:
- "ragionamento": Una stringa di testo che descrive il tuo processo di pensiero in una frase (il tuo "monologo interiore").
- "azione": Una stringa che rappresenta l'azione da compiere. Le azioni valide sono: "pick", "place", "done".
- "target_oggetto": Il nome dell'oggetto su cui agire (es. "CuboRosso"). Se l'azione è "done", questo campo è null.
- "target_posizione": Il nome della posizione dove agire (es. "AreaTarget"). Se l'azione è "pick" o "done", questo campo è null.

ESEMPIO:
---
Istruzione: "Metti il cubo rosso nella ciotola blu."
Scena: [Immagine di un cubo rosso e una ciotola blu]

Output JSON atteso:
{{
  "ragionamento": "Per mettere il cubo nella ciotola, devo prima prenderlo.",
  "azione": "pick",
  "target_oggetto": "CuboRosso",
  "target_posizione": null
}}
---

ORA TOCCA A TE. Data la seguente istruzione e immagine, genera l'output JSON.

Istruzione: "{istruzione_utente}"
Scena: [Immagine]
"""

# --- 2. FUNZIONE HELPER PER ESEGUIRE LE QUERY ---
def esegui_query_gemini(model_name, prompt_testo, immagine):
    """
    Inizializza un modello, esegue una query e restituisce la risposta.
    """
    try:
        print(f"\nInizializzazione del modello: {model_name}...")
        model = genai.GenerativeModel(model_name)

        contenuto_richiesta = [prompt_testo, immagine]

        print(f"Invio query a {model_name}...")
        response = model.generate_content(contenuto_richiesta)

        print(f"--- RISPOSTA GREZZA DI '{model_name}' ---")
        print(response.text)
        print("-" * 30)

        # <<< PASSAGGIO CHIAVE: PARSING DEL JSON >>>
        # Pulisci la risposta per assicurarti che sia solo JSON
        testo_risposta = response.text.strip().replace("```json", "").replace("```", "")

        try:
            # Converte la stringa JSON in un dizionario Python
            risposta_dict = json.loads(testo_risposta)

            print("--- RISPOSTA PARSATA (Dizionario Python) ---")
            print(f"Ragionamento: {risposta_dict.get('ragionamento')}")
            print(f"Azione: {risposta_dict.get('azione')}")
            print(f"Target Oggetto: {risposta_dict.get('target_oggetto')}")
            print(f"Target Posizione: {risposta_dict.get('target_posizione')}")

            # Qui potrai inserire la logica per chiamare le tue funzioni del robot
            # Esempio:
            # if risposta_dict.get('azione') == 'pick':
            #     pick(sim, risposta_dict.get('target_oggetto'))

        except json.JSONDecodeError:
            print("❌ Errore: Il modello non ha restituito un JSON valido.")

    except Exception as e:
        print(f"\n❌ Errore durante l'interrogazione del modello {model_name}. Dettagli: {e}")


# --- 3. FUNZIONE PRINCIPALE DEL PROGRAMMA ---
def main():
    """
    Contiene il loop principale per l'interazione con l'utente e il simulatore.
    """
    # Connessione a CoppeliaSim (una sola volta all'inizio)
    IS_COPPELIA = False
    if(IS_COPPELIA):
        try:
            print("Tentativo di connessione a CoppeliaSim...")
            client = RemoteAPIClient()
            sim = client.require('sim')
            vision_sensor_handle = sim.getObject('/Vision_sensor')
            print("✅ Connesso a CoppeliaSim e sensore di visione trovato.")
        except Exception as e:
            print(f"❌ Errore di connessione a CoppeliaSim. Assicurati che il simulatore sia in esecuzione. Dettagli: {e}")
            return

    # Ciclo principale (do-while)
    while True:
        # --- SCELTA DELLA MODALITÀ ---
        print("\n" + "=" * 50)
        print("SCEGLI UNA MODALITÀ:")
        print("1. Singolo Modello (Testa un modello alla volta)")
        print("2. Confronto (Interroga entrambi i modelli contemporaneamente)")
        print("3. Esci dal programma")
        print("=" * 50)

        scelta_modalita = input("Inserisci la tua scelta (1, 2 o 3): ")

        modelli_da_usare = []

        if scelta_modalita == '1':
            # --- SCELTA DEL SINGOLO MODELLO ---
            while True:
                print("\nSCEGLI QUALE MODELLO USARE:")
                print(f"1. {MODEL_ROBOTICS} (Specialista per la robotica)")
                print(f"2. {MODEL_GEMMA} (Generalista potente)")
                scelta_modello = input("Inserisci la tua scelta (1 o 2): ")

                if scelta_modello == '1':
                    modelli_da_usare.append(MODEL_ROBOTICS)
                    break
                elif scelta_modello == '2':
                    modelli_da_usare.append(MODEL_GEMMA)
                    break
                else:
                    print("❌ Scelta non valida. Riprova.")

        elif scelta_modalita == '2':
            # --- MODALITÀ CONFRONTO ---
            print("\nModalità Confronto selezionata. Verranno usati entrambi i modelli.")
            modelli_da_usare.extend([MODEL_ROBOTICS, MODEL_GEMMA])

        elif scelta_modalita == '3':
            # --- USCITA DAL PROGRAMMA ---
            print("Uscita dal programma. Arrivederci!")
            break

        else:
            print("❌ Scelta non valida. Inserisci 1, 2 o 3.")
            continue  # Torna all'inizio del loop

        # --- LOGICA DI ACQUISIZIONE E QUERY (comune a entrambe le modalità) ---
        # 1. Acquisisci immagine
        if(IS_COPPELIA):
            # Acquisisci l'immagine dal simulatore
            print("\nScattando una foto dal simulatore...")
            img, res = sim.getVisionSensorImg(vision_sensor_handle)
            if not img:
                print(
                    "❌ Fallimento: non è stato possibile catturare l'immagine. Assicurati che la simulazione sia in esecuzione.")
                continue

            img_np = np.frombuffer(img, dtype=np.uint8).reshape((res[1], res[0], 3))
            img_np = np.flipud(img_np)
            pil_image = Image.fromarray(img_np, 'RGB')
            print("✅ Immagine acquisita.")
        else:
            # Leggi immagine da file
            nome_file_immagine = "scatto_simulatore.png"
            print(f"\nCaricando l'immagine dal file: {nome_file_immagine}...")
            try:
                pil_image = Image.open(nome_file_immagine).convert('RGB')
                print("✅ Immagine caricata con successo.")
            except FileNotFoundError:
                print(
                    f"❌ Errore: File non trovato! Assicurati che '{nome_file_immagine}' sia nella stessa cartella dello script.")
                continue  # Torna al menu principale

        # 2. Chiedi all'utente il prompt testuale
        prompt_utente = input("\nInserisci la tua domanda per il modello (es. 'Cosa vedi?'): ")
        # Componi il prompt completo usando il template
        prompt_completo = master_prompt_template.format(istruzione_utente=prompt_utente)

        # 3. Esegui la query per ogni modello selezionato
        for nome_modello in modelli_da_usare:
            esegui_query_gemini(nome_modello, prompt_completo, pil_image)


# --- AVVIO DEL PROGRAMMA ---
if __name__ == "__main__":
    main()