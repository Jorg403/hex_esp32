import numpy as np
from lib.robotics.pose_generation.movement.gaits.gait_engine import GaitEngine

class TwoLegGaitEngine(GaitEngine):
    """
    Two-legs-at-a-time gait: three groups of two transverse (same fore-aft row) legs.
    Steps: front pair → middle pair → back pair → repeat.

    Groups:
      [0, 3]  front pair  (front-right + front-left)
      [1, 4]  middle pair (right-mid   + left-mid)
      [2, 5]  back pair   (back-right  + back-left)
    """
    leg_groups = [
        np.array([0, 3], dtype=np.int32),
        np.array([1, 4], dtype=np.int32),
        np.array([2, 5], dtype=np.int32),
    ]
