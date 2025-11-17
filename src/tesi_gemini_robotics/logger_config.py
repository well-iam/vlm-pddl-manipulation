# src/tesi_gemini_robotics/logger_config.py
import logging
import colorlog
import sys



def setup_logging(level=logging.WARNING):
    """
    Configura il logger principale (root logger) per l'intera applicazione.
    Va chiamata UNA SOLA VOLTA all'inizio dello script principale (main).
    """
    log_colors = {
        'DEBUG': 'white',
        'INFO': 'blue',
        'WARNING': 'yellow',
        'ERROR': 'red',
        'CRITICAL': 'bold_red,bg_white',  # Esempio: rosso grassetto su sfondo bianco
    }

    # log_format = "%(log_color)s[%(asctime)s][%(name)-25s][%(levelname)-8s] %(message)s"
    log_format = "%(log_color)s[%(asctime)s][%(name)s][%(levelname)s] %(message)s"
    date_format = '%H:%M:%S'

    formatter = colorlog.ColoredFormatter(
        log_format,
        datefmt=date_format,
        log_colors=log_colors,
        reset=True,  # Resetta il colore dopo ogni messaggio
        style='%'  # Stile di formattazione (default)
    )

    root_logger = logging.getLogger()

    # Imposta il livello di soglia globale sul logger radice
    root_logger.setLevel(level)

    # Rimuovi eventuali handler esistenti per evitare output doppi
    if root_logger.hasHandlers():
        root_logger.handlers.clear()
    # Crea un handler che invia i log alla console (sys.stdout)
    handler = logging.StreamHandler(sys.stdout)
    # Assegna il nostro nuovo formatter colorato all'handler
    handler.setFormatter(formatter)

    # Aggiungi il nostro handler configurato al logger radice
    root_logger.addHandler(handler)

    # SILENZIA I LOGGER ESTERNI RUMOROSI
    #
    # Ora che il nostro logger radice è impostato (es. a DEBUG),
    # diciamo a logger specifici di essere meno loquaci, impostando
    # il loro livello a WARNING. Questo nasconderà i loro messaggi
    # INFO e DEBUG, senza influenzare i nostri.

    # Lista dei logger da silenziare
    noisy_loggers = [
        "httpx",
        "httpcore",
        "httpcore.http11",
        "httpcore.connection",
        "google.api_core",
        "google.auth"
    ]

    for logger_name in noisy_loggers:
        logging.getLogger(logger_name).setLevel(logging.WARNING)

    # Messaggio di avvio
    logging.info(f"Logging colorato initializzato a livello: {logging.getLevelName(level)}")