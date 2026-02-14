# src/tesi_gemini_robotics/llm_interface.py

from google import genai
from google.genai import types
from google.genai.types import Content, Part
import json
import os
import logging
from PIL import Image
from dotenv import load_dotenv

load_dotenv()
logger = logging.getLogger(__name__)


class GeminiClient:
    """
    Manages interaction with a Gemini model, supporting textual or visual inputs
    for selecting the next robotic skill.
    """

    def __init__(self, model_name='models/gemini-robotics-er-1.5-preview', system_instruction=""):
        logger.info(f"Initializing GeminiClient with model: {model_name}...")
        try:
            # The API key is now handled automatically by the client
            # via the GOOGLE_API_KEY environment variable.
            # (Ensure you rename your environment variable from GEMINI_API_KEY)
            if not os.getenv("GEMINI_API_KEY"):
                raise ValueError("Environment variable GEMINI_API_KEY not set.")

            self.client = genai.Client()
            self.model_name = model_name
            # The model is now just a reference, not a heavy object
            self.model = self.client.models.get(model=self.model_name)

            # Save rules
            self.system_instruction = system_instruction

            self.chat_history = []  # We handle history internally
            logger.info(f"✅ Model {self.model_name} found and client initialized.")

        except Exception as e:
            logger.exception(f"❌ Error during Gemini client initialization: ")
            raise

    def start_chat(self):
        """
        Starts a new empty chat session for a robotic task.

        Returns:
            ChatSession: The started chat session object.
        """
        logger.debug("\n--- Starting new Robotic Chat session ---")
        self.chat_history = []
        # Initialize chat without initial history
        return

    def send_chat_message(self, prompt):
        """
        Sends a message to the chat, applying the system instruction.
        If skill_list is provided, it expects and validates a JSON.
        """
        logger.debug(f"""
        --- Sending message to Chat ---
          - User Prompt: {prompt}
        """)

        try:
            self.chat_history.append(Content(role='user', parts=[Part(text=prompt)]))

            # --- KEY MODIFICATION 2 ---
            # Pass both history and fixed system instruction
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=self.chat_history,
                config=types.GenerateContentConfig(
                    system_instruction=self.system_instruction),
            )

            self.chat_history.append(response.candidates[0].content)
            logger.debug(f"Response (raw) from model:\n{response.text}")
            return response

        except Exception as e:
            logger.exception(f"❌ Error during sending/receiving message: ")
            return None

    def ask_for_skill_json(self, high_level_goal, skill_list):
        """
        Sends a message to the chat expecting a JSON response
        for skill selection and validates it.

        Args:
            high_level_goal (str): The prompt asking for skill selection.
            skill_list (list[str]): List of valid skill names.

        Returns:
            dict | None: Parsed JSON dictionary or None in case of error.
        """
        logger.debug("--- Requesting Skill JSON from Chat ---")
        try:
            response = self.send_chat_message(high_level_goal)
            # We use the helper method we already had to parse and validate
            return self._parse_gemini_response(response, skill_list)

        except Exception as e:
            logger.exception(f"❌ Error during Skill JSON request: ")
            return None

    def _parse_json(self, response):
        """
        Private helper method to parse and validate the JSON response from Gemini.
        """
        try:
            json_text = response.text.strip()
            if json_text.startswith("```json"): json_text = json_text[7:]
            if json_text.endswith("```"): json_text = json_text[:-3]

            risposta_dict = json.loads(json_text)
            return risposta_dict

        except json.JSONDecodeError:
            logger.exception("❌ Parsing Error: Gemini did not return valid JSON.")
            return None
        except Exception as e:
            logger.exception(f"❌ Unexpected error during parsing: ")
            return None

    def _parse_gemini_response(self, response, skill_list):
        """
        Private helper method to parse and validate the JSON response from Gemini.
        """
        try:

            json_text = response.text.strip()
            if json_text.startswith("```json"): json_text = json_text[7:]
            if json_text.endswith("```"): json_text = json_text[:-3]

            risposta_dict = json.loads(json_text)

            # Validation
            required_keys = ["ragionamento", "skill_scelta", "argomenti"]
            if not all(k in risposta_dict for k in required_keys):
                raise ValueError(f"JSON does not contain required keys: {required_keys}.")
            if risposta_dict["skill_scelta"] not in skill_list:
                raise ValueError(
                    f"Selected skill '{risposta_dict['skill_scelta']}' is not among available ones: {skill_list}.")

            logger.debug(f"""
            --- Parsed Response ---
              Reasoning: {risposta_dict['ragionamento']}
              Selected Skill: {risposta_dict['skill_scelta']}
              Arguments: {risposta_dict['argomenti']}
            """)
            return risposta_dict

        except json.JSONDecodeError:
            logger.exception("❌ Parsing Error: Gemini did not return valid JSON.")
            return None
        except ValueError as ve:
            logger.exception(f"❌ Validation Error: ")
            return None
        except Exception as e:
            logger.exception(f"❌ Unexpected error during parsing: ")
            return None

    def get_object_list_from_image(self, image, available_objects_name):
        """
        Sends the image and the object list to Gemini for identification.
        """
        logger.debug("--- PHASE 1: Constrained Perception (Gemini) ---")

        # Format object list for the prompt
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
            logger.debug("Requesting object identification from Gemini...")
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=contenuto_richiesta)
            # print("FULL RESPONSE") #DEBUG
            # print(response.text)       #DEBUG
            testo_risposta = response.text.strip().replace("```json", "").replace("```", "")
            dati_json = json.loads(testo_risposta)
            logger.debug(testo_risposta)

            object_names = dati_json.get("oggetti_visti", [])
            # print(f"✅ Gemini identified: {object_names}")
            return object_names
        except Exception as e:
            logger.exception(f"❌ Error during image analysis with Gemini: ")
            return []

    def get_bounding_box_of(self, object, image: Image.Image):
        """
        Asks Gemini to find the bounding box of a specific object within an image.

        Args:
            object (str): Name of the object to find.
            image (PIL.Image.Image): Image of the current scene.

        Returns:
            dict | None: Dictionary with detection data or None.
        """
        print(f"\n--- {self.model_name}: Requesting bounding_box from IMAGE ---")

        # The prompt now refers to the image provided as input
        prompt_text = f"""
        Trova la bounding box per l'oggetto: {object}.

        Struttura JSON richiesta: 
        {{
          "target_object": "{object}", 
          "detected": true/false, 
          "box_2d": [ymin, xmin, ymax, xmax], 
          "description": "breve descrizione visiva dell'oggetto trovato per conferma"
        }}
        """

        try:
            # For multimodal models, input is a list [image, text]
            contenuto_richiesta = [image, prompt_text]  # Image first for best practice

            print("Sending image and prompt...")
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=contenuto_richiesta,
                config=types.GenerateContentConfig(
                    system_instruction=self.system_instruction),
            )
            return self._parse_json(response)

        except Exception as e:
            print(f"❌ Error during multimodal query to {self.model_name}: {e}")
            return None