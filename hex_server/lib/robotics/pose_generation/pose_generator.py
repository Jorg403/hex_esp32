import abc
import threading
import numpy as np
import lib.constants as consts
from lib.utils.coords_utils import leg_to_base, base_to_leg

class PoseGenerator(abc.ABC):
    def __init__(self, window_size):
        self.window_size = window_size
        self.pos = consts.INITIAL_POSITIONS.copy()

        self.tm_base_body = np.eye(4, dtype=np.float32)
        self.tm_body_base = np.eye(4, dtype=np.float32)
        
        # poses_base = leg_to_base(self.pos, self.tm_base_body)

        # stp = 2.5
        # poses_base[0] = poses_base[0]+np.array([stp, 0, 0])
        # poses_base[1] = poses_base[1]+np.array([-stp, 0, 0])
        # poses_base[2] = poses_base[2]+np.array([stp, 0, 0])

        # self.pos = base_to_leg(poses_base, self.tm_body_base)

        self.plane = 'xy'
        self.last_win_pos = [window_size // 2] * 3
        self.lock = threading.Lock()

    @abc.abstractmethod
    def update(self):
        """Update the pose based on input mechanism."""
        pass
