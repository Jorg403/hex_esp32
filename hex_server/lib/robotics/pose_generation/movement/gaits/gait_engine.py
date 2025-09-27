import abc
import numpy as np
from enum import Enum, auto
import lib.constants as consts

class State(Enum):
    IDLE = auto()
    UNIDLING = auto()
    WALKING = auto()
    IDLING = auto()

class GaitEngine(abc.ABC):
    def __init__(self):
        self.current_swing_group = 0
        self.phase_counter = 0
        self.leg_groups = []
        self.state = State.IDLE
        self.target_positions = consts.INITIAL_POSITIONS_BODY.copy()

    @abc.abstractmethod
    def get_step_targets(self, leg_positions: np.ndarray, direction_vec: np.ndarray, target_positions: np.ndarray, is_at_target: bool) -> dict:
        """Return target positions for legs in swing phase.
        Returns:
            dict: {leg_index: target_position (np.ndarray shape (3,))}
        """
        pass
