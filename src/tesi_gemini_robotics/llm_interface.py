# src/tesi_gemini_robotics/llm_interface.py

from google import genai
from google.genai import types
from google.genai.types import Content, Part
import json
import string
import os
import logging
from PIL import Image  # Necessario per il nuovo metodo

logger = logging.getLogger(__name__)

class GeminiClient:
    """
    Gestisce l'interazione con un modello Gemini, supportando input testuali o visivi
    per la selezione della prossima skill robotica.
    """

    def __init__(self, model_name='models/gemini-robotics-er-1.5-preview', system_instruction_template=""):
        logger.info(f"Inizializzazione GeminiClient con modello: {model_name}...")
        try:
            # La chiave API ora è gestita automaticamente dal client
            # tramite la variabile d'ambiente GOOGLE_API_KEY
            # (Assicurati di rinominare la tua variabile d'ambiente da GEMINI_API_KEY)
            if not os.getenv("GEMINI_API_KEY"):
                raise ValueError("Variabile d'ambiente GEMINI_API_KEY non impostata.")


            self.client = genai.Client()
            self.model_name = model_name
            # Il modello ora è solo un riferimento, non un oggetto pesante
            self.model = self.client.models.get(model=self.model_name)

            # Salva le regole
            self.system_instruction_template = system_instruction_template
            self.system_instruction = None

            self.chat_history = []  # Gestiamo la cronologia internamente
            logger.info(f"✅ Modello {self.model_name} trovato e client inizializzato.")

        except Exception as e:
            logger.exception(f"❌ Errore durante l'inizializzazione del client Gemini: ")
            raise

    def update_system_instruction(self, **kwargs):
        """
        Aggiorna l'istruzione di sistema formattando il template interno
        con i dati forniti. Esegue la validazione dei segnaposto.

        Uso: client.update_system_instruction(dynamic_world_state=my_json)
        """
        try:
            # 1. VALIDAZIONE: Controlla quali campi si aspetta il template
            # string.Formatter().parse restituisce tuple (literal_text, field_name, format_spec, conversion)
            # Noi siamo interessati solo a field_name (es. 'dynamic_world_state')
            keys_needed = [fname for _, fname, _, _ in string.Formatter().parse(self.system_instruction_template) if
                           fname]

            # Controlla se tutti i campi necessari sono presenti in kwargs
            missing_keys = [key for key in keys_needed if key not in kwargs]

            if missing_keys:
                error_msg = f"Dati mancanti per il template System Instruction: {missing_keys}"
                logger.error(f"❌ {error_msg}")
                raise ValueError(error_msg)

            # 2. FORMATTAZIONE INTERNA
            # Usiamo l'unpacking (**kwargs) per passare i dati al format
            new_instruction = self.system_instruction_template.format(**kwargs)

            # 3. AGGIORNAMENTO STATO
            self.system_instruction = new_instruction

            # (Opzionale) Logga l'aggiornamento (solo i primi 100 caratteri per pulizia)
            logger.debug(f"  - System Instruction aggiornata internamente. \n {new_instruction}")

        except Exception as e:
            print(f"❌ Errore critico nell'aggiornamento della System Instruction: {e}")
            raise e

    def start_chat(self):
        """
        Avvia una nuova sessione di chat vuota per un task robotico.

        Returns:
            ChatSession: L'oggetto chat session avviato.
        """
        logger.debug("\n--- Avvio nuova sessione di Chat Robotica ---")
        self.chat_history = []
        # Inizializza la chat senza cronologia iniziale
        return

    def send_chat_message(self, prompt):
        """
        Invia un messaggio alla chat, applicando l'istruzione di sistema.
        Se skill_list è fornito, si aspetta e valida un JSON.
        """
        logger.debug(f"""
        --- Invio messaggio alla Chat ---
          - Prompt Utente: {prompt}
        """)

        try:
            self.chat_history.append(Content(role='user', parts=[Part(text=prompt)]))

            # --- MODIFICA CHIAVE 2 ---
            # Passiamo sia la cronologia che l'istruzione di sistema fissa
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=self.chat_history,
                config=types.GenerateContentConfig(
                    system_instruction=self.system_instruction),
            )

            self.chat_history.append(response.candidates[0].content)
            logger.debug(f"Risposta (grezza) del modello:\n{response.text}")
            return response

        except Exception as e:
            logger.exception(f"❌ Errore durante l'invio/ricezione del messaggio: ")
            return None

    def ask_for_skill_json(self, high_level_goal, skill_list):
        """
        Invia un messaggio alla chat aspettandosi una risposta JSON
        per la selezione della skill e la valida.

        Args:
            high_level_goal (str): Il prompt che chiede la selezione della skill.
            skill_list (list[str]): Lista dei nomi delle skill valide.

        Returns:
            dict | None: Dizionario JSON parsato o None in caso di errore.
        """
        logger.debug("--- Richiesta Skill JSON alla Chat ---")
        try:
            response = self.send_chat_message(high_level_goal)
            # Usiamo il metodo helper che già avevamo per parsare e validare
            return self._parse_gemini_response(response, skill_list)

        except Exception as e:
            logger.exception(f"❌ Errore durante la richiesta Skill JSON: ")
            return None

    def _parse_gemini_response(self, response, skill_list):
        """
        Metodo helper privato per parsare e validare la risposta JSON da Gemini.
        """
        try:

            json_text = response.text.strip()
            if json_text.startswith("```json"): json_text = json_text[7:]
            if json_text.endswith("```"): json_text = json_text[:-3]

            risposta_dict = json.loads(json_text)

            # Validazione
            required_keys = ["ragionamento", "skill_scelta", "argomenti"]
            if not all(k in risposta_dict for k in required_keys):
                raise ValueError(f"JSON non contiene le chiavi richieste: {required_keys}.")
            if risposta_dict["skill_scelta"] not in skill_list:
                raise ValueError(
                    f"Skill scelta '{risposta_dict['skill_scelta']}' non è tra quelle disponibili: {skill_list}.")

            logger.debug(f"""
            --- Risposta Parsata ---
              Ragionamento: {risposta_dict['ragionamento']}
              Skill Scelta: {risposta_dict['skill_scelta']}
              Argomenti: {risposta_dict['argomenti']}
            """)
            return risposta_dict

        except json.JSONDecodeError:
            logger.exception("❌ Errore Parsing: Gemini non ha restituito un JSON valido.")
            return None
        except ValueError as ve:
            logger.exception(f"❌ Errore Validazione: ")
            return None
        except Exception as e:
            logger.exception(f"❌ Errore imprevisto durante il parsing: ")
            return None

    def get_next_skill_from_text(self, scene_description, high_level_goal, skill_list, available_objects):
        """
        Chiede a Gemini di scegliere la prossima skill basandosi su una DESCRIZIONE TESTUALE.

        Args:
            scene_description (str): Descrizione testuale della scena.
            high_level_goal (str): Obiettivo finale dell'utente.
            skill_list (list[str]): Nomi delle skill disponibili.

        Returns:
            dict | None: Dizionario con la skill scelta o None.
        """
        print(f"\n--- {self.model_name}: Richiesta skill da TESTO ---")
        skill_options_str = ", ".join([f"'{s}'" for s in skill_list])
        object_list_str = ", ".join(available_objects)

        prompt = f"""
        Sei un pianificatore robotico. Scegli la prossima skill da eseguire per raggiungere l'obiettivo, basandoti sulla descrizione testuale della scena e sulla lista di skill.
        Gli argomenti della skill sono oggetti della scena che puoi scegliere solo tra la seguente lista: [{object_list_str}].

        Descrizione Scena: "{scene_description}"
        Obiettivo Finale: "{high_level_goal}"
        Skill Disponibili: [{skill_options_str}]

        Analizza la situazione. Scegli UNA skill dalla lista fornita.

        Rispondi SEMPRE E SOLO con un oggetto JSON valido con chiavi "ragionamento", "skill_scelta", "argomenti".

        ESEMPIO DI OUTPUT:
        {{
          "ragionamento": "L'obiettivo è afferrare il cilindro. La skill 'pick_and_hold' è la più adatta.",
          "skill_scelta": "pick_and_hold",
          "argomenti": ["Cilinder"] 
        }}
        """
        print(prompt)
        try:
            print("Invio prompt testuale...")
            # Per i modelli text-only o se non serve visione, l'input è solo il prompt
            response = self.model.generate_content(prompt)
            return self._parse_gemini_response(response, skill_list)

        except Exception as e:
            print(f"❌ Errore durante la query testuale a {self.model_name}: {e}")
            return None

    def get_next_skill_from_image(self, image: Image.Image, high_level_goal, skill_list, available_objects):
        """
        Chiede a Gemini di scegliere la prossima skill basandosi su un'IMMAGINE.

        Args:
            image (PIL.Image.Image): Immagine della scena attuale.
            high_level_goal (str): Obiettivo finale dell'utente.
            skill_list (list[str]): Nomi delle skill disponibili.

        Returns:
            dict | None: Dizionario con la skill scelta o None.
        """
        print(f"\n--- {self.model_name}: Richiesta skill da IMMAGINE ---")
        skill_options_str = ", ".join([f"'{s}'" for s in skill_list])
        object_list_str = ", ".join(available_objects)

        # Il prompt ora si riferisce all'immagine fornita come input
        prompt_text = f"""
        Sei un pianificatore robotico. Osserva l'immagine fornita e scegli la prossima skill più appropriata da eseguire per raggiungere l'obiettivo, selezionandola dalla lista di skill disponibili.
        Gli argomenti della skill sono oggetti della scena che puoi scegliere solo tra la seguente lista: [{object_list_str}].
        
        
        Obiettivo Finale: "{high_level_goal}"
        Skill Disponibili: [{skill_options_str}]

        Analizza l'immagine e l'obiettivo. Scegli UNA skill dalla lista fornita.

        Rispondi SEMPRE E SOLO con un oggetto JSON valido con chiavi "ragionamento", "skill_scelta", "argomenti".

        ESEMPIO DI OUTPUT:
        {{
          "ragionamento": "Dall'immagine vedo un cubo rosso. Per afferrarlo, userò 'pick_and_hold'.",
          "skill_scelta": "pick_and_hold",
          "argomenti": ["cubo_rosso"] 
        }}
        """

        try:
            # Per i modelli multimodali, l'input è una lista [immagine, testo]
            contenuto_richiesta = [image, prompt_text]  # Immagine prima per best practice

            print("Invio immagine e prompt...")
            response = self.model.generate_content(contenuto_richiesta)
            return self._parse_gemini_response(response, skill_list)

        except Exception as e:
            print(f"❌ Errore durante la query multimodale a {self.model_name}: {e}")
            return None

    def get_object_list_from_image(self, image, available_objects_name):
        """
        Invia l'immagine e la lista di oggetti a Gemini per l'identificazione.
        """
        logger.debug("--- FASE 1: Percezione Vincolata (Gemini) ---")

        # Formatta la lista di oggetti per il prompt
        object_list_str = ", ".join(available_objects_name)

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
            logger.debug("Richiesta di identificazione oggetti a Gemini...")
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=contenuto_richiesta)
            # print("RISPOSTA COMPLETA") #DEBUG
            # print(response.text)       #DEBUG
            testo_risposta = response.text.strip().replace("```json", "").replace("```", "")
            dati_json = json.loads(testo_risposta)
            logger.debug(testo_risposta)

            object_names = dati_json.get("oggetti_visti", [])
            # print(f"✅ Gemini ha identificato: {object_names}")
            return object_names
        except Exception as e:
            logger.exception(f"❌ Errore durante l'analisi dell'immagine con Gemini: ")
            return []