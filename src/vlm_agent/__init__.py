# Espone le classi principali per un accesso facile
from .core.logger_config import setup_logging
from .core.prompt_templates import *
from .core.task_executor import TaskExecutor
from .core.llm_interface import GeminiClient
from .implementations.coppelia.coppeliasim_robot import CoppeliaSimRobot
from .implementations.coppelia.coppeliasim_setup import connect_to_sim