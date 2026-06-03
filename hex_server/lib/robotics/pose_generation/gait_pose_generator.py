import numpy as np
from lib.utils.coords_utils import leg_to_base, base_to_leg
from lib.robotics.pose_generation.pose_generator import PoseGenerator
from lib.robotics.pose_generation.movement.gaits.gait_engine import State
from lib.robotics.pose_generation.movement.gaits.gait_engine_constructor import gait_engine_constructor
from lib.robotics.pose_generation.movement.trajectory_planner import generate_new_positions
import lib.controllers.walk_PS_controller as PS_controller
import lib.constants as consts

# Converts rotation_speed (from PS rx/10, range ±0.1) to an equivalent speed
# scalar so rotation-only movement still drives the trajectory planner.
_ROT_TO_SPEED = 10.0


class GaitPoseGenerator(PoseGenerator):
    def __init__(self, window_size, gait_engine):
        super().__init__(window_size)
        self.current_gait_name = gait_engine
        self.gait_engine = gait_engine_constructor(gait_engine)
        self.walk_controller = PS_controller.WalkPSController()

        self.current_direction = np.zeros(2, dtype=np.float32)
        self.current_speed = 0.0
        self.rotation_speed = 0.0
        self.target_positions = consts.INITIAL_POSITIONS.copy()

    def set_gait_engine(self, gait_name: str):
        """Hot-swap gait; robot returns to home before the new gait starts."""
        if gait_name == self.current_gait_name:
            return
        self.current_gait_name = gait_name
        self.gait_engine = gait_engine_constructor(gait_name)
        # Reset targets to home so the new engine always starts from a clean state.
        self.target_positions = consts.INITIAL_POSITIONS.copy()

    def update_from_input(self):
        inputs = self.walk_controller.get_motion_command()
        if inputs is None:
            return

        self.current_direction = inputs["direction"]
        self.current_speed = inputs["speed"]
        self.rotation_speed = inputs["rotation_speed"]
        fwd_back   = inputs["fwd_back"]
        left_right = inputs["left_right"]
        up_down    = inputs["up_down"]
        roll       = inputs["roll"]
        pitch      = inputs["pitch"]
        yaw        = inputs["yaw"]

        # Handle gait switching signalled by the controller
        gait = inputs.get("gait")
        if gait:
            self.set_gait_engine(gait)

        # Preserve leg positions in base frame before changing body tilt
        pos_base = leg_to_base(self.pos, self.tm_base_body)

        cr = np.cos(np.radians(roll));  sr = np.sin(np.radians(roll))
        cp = np.cos(np.radians(pitch)); sp = np.sin(np.radians(pitch))
        cy = np.cos(np.radians(yaw));   sy = np.sin(np.radians(yaw))

        self.tm_body_base = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr, fwd_back],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, left_right],
            [-sp,   cp*sr,            cp*cr,            up_down],
            [0,     0,                0,                1]
        ], dtype=np.float32)
        self.tm_base_body = np.linalg.inv(self.tm_body_base)

        self.pos = base_to_leg(pos_base, self.tm_body_base)

    def update(self):
        """Main tick: read input → gait engine → trajectory planner."""
        self.update_from_input()

        moving = (np.linalg.norm(self.current_direction) > 0.01
                  or abs(self.rotation_speed) > 0.001)

        if self.gait_engine.state == State.IDLE and not moving:
            return self.pos

        # is_at_target: compare in leg frame (both arrays are in leg frame)
        is_at_target = np.allclose(self.pos, self.target_positions, atol=0.1)

        # Gait engine operates in body frame
        pos_body = leg_to_base(self.pos, self.tm_base_body)

        new_tgt_body = self.gait_engine.get_step_targets(
            pos_body,
            self.current_direction,
            self.rotation_speed,
            is_at_target,
        )

        self.target_positions = base_to_leg(new_tgt_body, self.tm_body_base)

        # Effective speed: translation speed, or rotation-derived speed if translating
        effective_speed = max(self.current_speed, abs(self.rotation_speed) * _ROT_TO_SPEED)
        speed = (effective_speed
                 if self.gait_engine.state != State.IDLING
                 else consts.IDLING_SPEED)
        is_air = np.zeros(6, dtype=bool)
        airs = self.gait_engine.leg_groups[self.gait_engine.current_swing_group]
        is_air[airs] = True

        self.pos = generate_new_positions(self.pos, self.target_positions, is_air, speed=speed)
        return self.pos
