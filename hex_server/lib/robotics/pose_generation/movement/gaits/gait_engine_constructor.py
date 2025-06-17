from lib.robotics.pose_generation.movement.gaits.tripod_gait import TripodGaitEngine

def gait_engine_constructor(mode: str):
    if mode == 'tripod':
        return TripodGaitEngine()
