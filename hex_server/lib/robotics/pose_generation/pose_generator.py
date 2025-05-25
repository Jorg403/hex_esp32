import abc
import threading
import numpy as np

class PoseGenerator(abc.ABC):
    def __init__(self, window_size):
        self.window_size = window_size
        self.pos = np.array([15.0, 0.0, 6.0], dtype=np.float32)
        self.plane = 'xy'
        self.last_win_pos = [window_size // 2] * 3
        self.lock = threading.Lock()

    @abc.abstractmethod
    def update(self):
        """Update the pose based on input mechanism."""
        pass
