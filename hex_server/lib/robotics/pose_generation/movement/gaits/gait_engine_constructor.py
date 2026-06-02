from lib.robotics.pose_generation.movement.gaits.tripod_gait import TripodGaitEngine
from lib.robotics.pose_generation.movement.gaits.dog_gait import DogGaitEngine
from lib.robotics.pose_generation.movement.gaits.monopod_gait import MonopodGaitEngine
from lib.robotics.pose_generation.movement.gaits.ripple_gait import RippleGaitEngine

_GAIT_MAP = {
    'tripod':  TripodGaitEngine,
    'ripple':  RippleGaitEngine,
    'monopod': MonopodGaitEngine,
    'dog':    DogGaitEngine,
}

GAIT_NAMES = list(_GAIT_MAP.keys())

def gait_engine_constructor(mode: str):
    cls = _GAIT_MAP.get(mode)
    if cls is None:
        raise ValueError(f"Unknown gait '{mode}'. Available: {GAIT_NAMES}")
    return cls()
