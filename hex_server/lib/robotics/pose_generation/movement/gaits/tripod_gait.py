import numpy as np
from lib.robotics.pose_generation.movement.gaits.gait_engine import GaitEngine

class TripodGaitEngine(GaitEngine):
    """
    Classic tripod gait: two alternating groups of three legs.
    Groups [0,2,4] and [1,3,5] form stable triangles at all times.
    """
    leg_groups = [
        np.array([0, 2, 4], dtype=np.int32),
        np.array([1, 3, 5], dtype=np.int32),
    ]
