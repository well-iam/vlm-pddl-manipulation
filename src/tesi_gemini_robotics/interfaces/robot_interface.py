# src/tesi_gemini_robotics/interfaces/robot_interface.py
from abc import ABC, abstractmethod

class RobotInterface(ABC):

    @abstractmethod
    def perform_pick_and_hold(self, object_name: str) -> bool:
        """
        Esegue la sequenza completa di presa:
        pre-grasp -> approach -> close gripper -> attach (sim only) -> lift.
        """
        pass

    @abstractmethod
    def perform_place(self, object_name, location_name: str) -> bool:
        """
        Esegue la sequenza completa di rilascio:
        move to location -> descend -> detach (sim only) -> open gripper -> retreat.
        """
        pass