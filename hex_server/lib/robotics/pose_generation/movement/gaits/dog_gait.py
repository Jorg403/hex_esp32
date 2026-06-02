import numpy as np
from lib.robotics.pose_generation.movement.gaits.gait_engine import GaitEngine

class DogGaitEngine(GaitEngine):
    """
    Ripple gait: three groups of two diagonal legs stepping in sequence.
    At any time 4 legs are grounded → smoother and more stable than tripod.

    Groups (diagonal pairs):
      [0, 5]  front-right + back-left
      [2, 3]  back-right  + front-left
      [1, 4]  right-mid   + left-mid
    """
    leg_groups = [
        np.array([0, 5], dtype=np.int32),
        np.array([2, 3], dtype=np.int32),
    ]

    dead_legs = np.array([1, 4], dtype=np.int32)
