import numpy as np
from lib.utils.coords_utils import leg_to_base, base_to_leg
from lib.robotics.pose_generation.pose_generator import PoseGenerator
from lib.robotics.pose_generation.movement.gaits.gait_engine import GaitEngine
from lib.robotics.pose_generation.movement.gaits.gait_engine_constructor import gait_engine_constructor
from lib.robotics.pose_generation.movement.trajectory_planner import generate_new_positions
import pdb
from lib.inputs.slider_input import SliderInput

# import logging
# logging.debug(f"Swing legs: {list(target_positions.keys())}")

class GaitPoseGenerator(PoseGenerator):
    def __init__(self, window_size, gait_engine):
        super().__init__(window_size)
        self.gait_engine = gait_engine_constructor(gait_engine)
        self.sliders = SliderInput(n_sliders=8, slider_names=[
            "Forward/Backward", "Left/Right", "Up/Down",
            "Roll", "Pitch", "Yaw", "vx", "vy"
        ])
        self.current_direction = np.array([0.0, 0.0], dtype=np.float32)
        self.current_speed = 0
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

    def update_from_sliders(self, sliders):
        """Update movement parameters from slider input."""
        fwd_back = sliders[0] / 10      # -10 to 10
        left_right = sliders[1] / 10    # -10 to 10
        up_down = sliders[2] / 10       # -10 to 10

        roll = sliders[3] / 100 * 15    # -15 to 15 degrees
        pitch = sliders[4] / 100 * 15   # -15 to 15 degrees
        yaw = sliders[5] / 100 * 15     # -15 to 15 degrees

        vx = sliders[6] / 100      # -10 to 10 cm/s
        vy = sliders[7] / 100      # -10 to 10 cm/s

        self.set_direction(np.array([vx, vy], dtype=np.float32))
        self.current_speed = min(self.current_speed, 1.0)

        pos_base = leg_to_base(self.pos, self.tm_base_body)

        # Update body transformation matrix
        cr = np.cos(np.radians(roll)); sr = np.sin(np.radians(roll))
        cp = np.cos(np.radians(pitch)); sp = np.sin(np.radians(pitch))
        cy = np.cos(np.radians(yaw)); sy = np.sin(np.radians(yaw))

        self.tm_body_base = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr, fwd_back],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, left_right],
            [-sp,   cp*sr,            cp*cr,            up_down],
            [0,     0,                0,                1]
        ], dtype=np.float32)
        self.tm_base_body = np.linalg.inv(self.tm_body_base)

        # self.pos = base_to_leg(pos_base, self.tm_body_base)
        # round to three decimals
        self.pos = base_to_leg(pos_base, self.tm_body_base)
        

    def update(self):
        """Update per-leg foot positions based on gait + trajectory planning."""

        # update direction with input
        self.sliders.poll()  # mantiene la ventana viva
        sliders = self.sliders.get_state()
        if sliders is not None:
            self.update_from_sliders(sliders)

        if self.current_speed == 0:
            return self.pos

        # print("Target positions before gait engine:\n", self.target_positions[0:1])
        # print("Current positions before gait engine:\n", self.pos[0:1])

        self.target_positions = base_to_leg(self.gait_engine.get_step_targets(leg_to_base(self.pos, self.tm_base_body), self.current_direction, self.target_positions, (self.target_positions == self.pos).all()), self.tm_body_base)

        # print("Target positions after gait engine:\n", self.target_positions[0:1])

        self.pos = generate_new_positions(
            self.pos,
            self.target_positions,
            speed=self.current_speed
        )
        # print("Current positions after trajectory planning:\n", self.pos[0:1])
        # import pdb; pdb.set_trace()
        # print("Current positions:\n", self.pos)
        # print("Target positions:\n", self.target_positions)

        return self.pos

        
