import numpy as np
from lib.robotics.pose_generation.movement.gaits.gait_engine import GaitEngine

class TripodGaitEngine(GaitEngine):
    def __init__(self):
        super().__init__()

        self.leg_groups = np.array([[0, 2, 4], [1, 3, 5]], dtype=np.int32)
        self.current_swing_group = 0  # 0 or 1

        stp = 5
        self.hardcoded_movements = np.array([
            [0.0, 0.0, 0.0],
            [-stp, 0.0, 0.0],
            [0.0, 0.0, 0.0],

            [0.0, 0.0, 2.0],
            [stp, 0.0, 0.0],
            [0.0, 0.0, -2.0]
        ], dtype=np.float32)
        self.n_movements = len(self.hardcoded_movements)

    def get_step_targets(self, leg_positions: np.ndarray, direction_vec: np.ndarray) -> dict:
        targets = np.zeros((6, 3), dtype=np.float32)
        targets[self.leg_groups[self.current_swing_group]] = leg_positions[self.leg_groups[self.current_swing_group]] + self.hardcoded_movements[(self.n_movements//2*self.current_swing_group + self.phase_counter) % self.n_movements]
        targets[self.leg_groups[1 - self.current_swing_group]] = leg_positions[self.leg_groups[1 - self.current_swing_group]] + self.hardcoded_movements[(self.n_movements//2*self.current_swing_group + self.phase_counter + 3) % self.n_movements]
        self.phase_counter = (self.phase_counter + 1) % self.n_movements

        if self.phase_counter == 0:
            self.current_swing_group = 1 - self.current_swing_group

        return targets
