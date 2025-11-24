import math
import json
import logging
import time

logger = logging.getLogger(__name__)



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