import numpy as np
from lib.robotics.pose_generation.movement.gaits.gait_engine import GaitEngine

class MonopodGaitEngine(GaitEngine):
    """
    Monopod gait: one at a time.
    At any time 5 legs are grounded → smoother and more stable than tripod.

    Groups (diagonal pairs):
      [0]  front-right
      [1]  front-left
      [2]  middle-right
      [3]  middle-left
      [4]  rear-right
      [5]  rear-left
    """
    leg_groups = [
        np.array([0], dtype=np.int32),
        np.array([1], dtype=np.int32),
        np.array([2], dtype=np.int32),
        np.array([3], dtype=np.int32),
        np.array([4], dtype=np.int32),
        np.array([5], dtype=np.int32),
    ]
