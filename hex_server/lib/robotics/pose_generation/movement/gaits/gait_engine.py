import abc
import numpy as np

class GaitEngine(abc.ABC):
    def __init__(self):
        self.phase_counter = 0

    @abc.abstractmethod
    def get_step_targets(self, leg_positions: np.ndarray, direction_vec: np.ndarray) -> dict:
        """Return target positions for legs in swing phase.
        Returns:
            dict: {leg_index: target_position (np.ndarray shape (3,))}
        """
        pass
