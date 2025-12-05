
from abc import ABC, abstractmethod


class Controller(ABC):
    def __init__(self):
        self._state = {
            "success": False,
            "failure": False,
            "movement_enabled": False,
            "controller_on": True,
        }

    def get_action_space(self):
        return self.action_space
    
    def get_gripper_action_space(self):
        return self.gripper_action_space

    @abstractmethod
    def forward(self, obs: dict) -> dict:
        """Forward the observation to the controller and produce an action."""
        pass

    @abstractmethod
    def reset_state(self):
        """Reset the state of the controller."""
        pass

    @abstractmethod
    def register_key(self, key):
        """Register a key press from the Runner window."""
        pass

    @abstractmethod
    def get_info(self) -> dict:
        """Get the current state of the controller."""
        pass

    @abstractmethod
    def close(self):
        """Close the controller."""
        pass