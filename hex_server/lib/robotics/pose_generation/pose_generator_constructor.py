from lib.robotics.pose_generation.mouse_pose_generator import MousePoseGenerator
from lib.robotics.pose_generation.controller_pose_generator import ControllerPoseGenerator

def create_pose_generator(mode: str, window_size: int = 500):
    if mode == 'mouse':
        return MousePoseGenerator(window_size)
    elif mode == 'controller':
        return ControllerPoseGenerator(window_size)
    else:
        raise ValueError(f"Unsupported pose generator mode: '{mode}'")
