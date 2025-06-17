import numpy as np
from lib.utils.coords_utils import leg_to_base, base_to_leg
from lib.robotics.pose_generation.pose_generator import PoseGenerator
from lib.robotics.pose_generation.movement.gaits.gait_engine import GaitEngine
from lib.robotics.pose_generation.movement.gaits.gait_engine_constructor import gait_engine_constructor
from lib.robotics.pose_generation.movement.trajectory_planner import generate_new_positions


# import logging
# logging.debug(f"Swing legs: {list(target_positions.keys())}")

class GaitPoseGenerator(PoseGenerator):
    def __init__(self, window_size, gait_engine):
        super().__init__(window_size)
        self.gait_engine = gait_engine_constructor(gait_engine)
        self.current_direction = np.array([0.0, 0.0], dtype=np.float32)
        self.current_speed = 1.0
        self.target_positions = np.copy(self.pos)

    def set_gait_engine(self, gait_engine: GaitEngine):
        """Set the gait engine to use for generating leg positions."""
        self.gait_engine = gait_engine
        self.target_positions = np.copy(self.pos)

    def set_direction(self, direction_vec: np.ndarray):
        """Set 2D direction of movement in the body plane."""
        self.current_speed = np.linalg.norm(direction_vec)
        if self.current_speed > 0:
            self.current_direction = direction_vec / self.current_speed
        else:
            self.current_direction = np.zeros_like(direction_vec)

    def update(self):
        """Update per-leg foot positions based on gait + trajectory planning."""
        if self.current_speed == 0: return
        
        if (self.target_positions == self.pos).all():
            self.target_positions = base_to_leg(self.gait_engine.get_step_targets(leg_to_base(self.pos), self.current_direction))
    
        self.pos = generate_new_positions(
            self.pos,
            self.target_positions,
            speed=self.current_speed
        )
