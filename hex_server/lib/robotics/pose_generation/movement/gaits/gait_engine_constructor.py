from lib.robotics.pose_generation.movement.gaits.tripod_gait import TripodGaitEngine
from lib.robotics.pose_generation.movement.gaits.ripple_gait import RippleGaitEngine
from lib.robotics.pose_generation.movement.gaits.two_leg_gait import TwoLegGaitEngine

_GAIT_MAP = {
    'tripod':  TripodGaitEngine,
    'ripple':  RippleGaitEngine,
    'two_leg': TwoLegGaitEngine,
}

GAIT_NAMES = list(_GAIT_MAP.keys())

def gait_engine_constructor(mode: str):
    cls = _GAIT_MAP.get(mode)
    if cls is None:
        raise ValueError(f"Unknown gait '{mode}'. Available: {GAIT_NAMES}")
    return cls()
