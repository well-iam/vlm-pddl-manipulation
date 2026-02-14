# src/tesi_gemini_robotics/logger_config.py
import logging
import colorlog
import sys

def setup_logging(level=logging.WARNING):
    """
    Configures the root logger for the entire application.
    Must be called ONLY ONCE at the beginning of the main script.
    """
    log_colors = {
        'DEBUG': 'white',
        'INFO': 'blue',
        'WARNING': 'yellow',
        'ERROR': 'red',
        'CRITICAL': 'bold_red,bg_white',  # Example: bold red on white background
    }

    # log_format = "%(log_color)s[%(asctime)s][%(name)-25s][%(levelname)-8s] %(message)s"
    log_format = "%(log_color)s[%(asctime)s][%(name)s][%(levelname)s] %(message)s"
    date_format = '%H:%M:%S'

    formatter = colorlog.ColoredFormatter(
        log_format,
        datefmt=date_format,
        log_colors=log_colors,
        reset=True,  # Resets color after each message
        style='%'  # Formatting style (default)
    )

    root_logger = logging.getLogger()

    # Set the global threshold level on the root logger
    root_logger.setLevel(level)

    # Remove existing handlers to avoid double output
    if root_logger.hasHandlers():
        root_logger.handlers.clear()
    # Create a handler that sends logs to console (sys.stdout)
    handler = logging.StreamHandler(sys.stdout)
    # Assign our new colored formatter to the handler
    handler.setFormatter(formatter)

    # Add our configured handler to the root logger
    root_logger.addHandler(handler)

    # SILENCE NOISY EXTERNAL LOGGERS
    #
    # Now that our root logger is set (e.g., to DEBUG),
    # we tell specific loggers to be less talkative by setting
    # their level to WARNING. This will hide their INFO and DEBUG
    # messages without affecting ours.

    # List of loggers to silence
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

    # Startup message
    logging.info(f"Colored logging initialized at level: {logging.getLevelName(level)}")