# src/tesi_gemini_robotics/vlm_interface.py

from google import genai
from google.genai import types
from google.genai.types import Content, Part
import os
import logging

logger = logging.getLogger(__name__)


class GeminiClient:
    """
    Manages interaction with a Gemini model, supporting textual or visual inputs
    for selecting the next robotic skill.
    """

    def __init__(self, model_name='models/gemini-robotics-er-1.5-preview', system_instruction=""):
        # logger.debug(f"Initializing GeminiClient with model: {model_name}...")
        try:
            # The API key is now automatically managed by the client
            # via the GOOGLE_API_KEY environment variable
            # (Ensure you rename your environment variable from GEMINI_API_KEY)
            if not os.getenv("GEMINI_API_KEY"):
                raise ValueError("GEMINI_API_KEY environment variable not set.")

            self.client = genai.Client()
            self.model_name = model_name
            # The model is now just a reference, not a heavy object
            self.model = self.client.models.get(model=self.model_name)

            # Save rules
            self.system_instruction = system_instruction

            self.chat_history = []  # We manage history internally
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
        # logger.debug("Starting new session of Robotics Chat")
        self.chat_history = []
        # Initialize the chat without initial history
        return

    # JPEG for efficiency, PNG for maximum fidelity
    def send_chat_message(self, prompt, image_bytes=None, mime_type='image/jpeg'):
        """
        Sends a message to the chat, applying the system instruction.
        Supports multimodal input (Text + Optional Image).
        """
        log_msg = f"User Prompt: \n{prompt}"
        if image_bytes:
            log_msg += " [ + Attached image ]"

        logger.debug(f"""
        --- Sending message to the Chat ---
          - {log_msg}
        """)

        try:
            # 1. Dynamic construction of message parts
            current_request_parts = [Part(text=prompt)]

            if image_bytes:
                # Create Blob with received bytes
                image_part = Part(
                    inline_data=types.Blob(
                        mime_type=mime_type,
                        data=image_bytes
                    )
                )
                current_request_parts.append(image_part)

                # Create content for API call
            contents_for_api = list(self.chat_history)
            contents_for_api.append(Content(role='user', parts=current_request_parts))

            # Pass both the history and the fixed system instruction
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=contents_for_api,
                config=types.GenerateContentConfig(
                    system_instruction=self.system_instruction),
            )

            # Update history (WITHOUT IMAGE)
            self.chat_history.append(Content(role='user', parts=[Part(text=prompt)]))
            self.chat_history.append(response.candidates[0].content)

            logger.debug(f"Raw answer:\n{response.text}")
            return response

        except Exception as e:
            logger.exception(f"❌ Error during message send/receive: ")
            return None