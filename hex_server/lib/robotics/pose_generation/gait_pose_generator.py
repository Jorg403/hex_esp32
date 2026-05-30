import numpy as np
from lib.utils.coords_utils import leg_to_base, base_to_leg, world_to_base, base_to_world
from lib.robotics.pose_generation.pose_generator import PoseGenerator
from lib.robotics.pose_generation.movement.gaits.gait_engine import GaitEngine, State
from lib.robotics.pose_generation.movement.gaits.gait_engine_constructor import gait_engine_constructor
from lib.robotics.pose_generation.movement.trajectory_planner import generate_new_positions
import pdb
from lib.inputs.slider_input import SliderInput
import lib.controllers.walk_PS_controller as PS_controller
import lib.controllers.walk_slider_controller as slider_controller
import lib.constants as consts


# import logging
# logging.debug(f"Swing legs: {list(target_positions.keys())}")

class GaitPoseGenerator(PoseGenerator):
    def __init__(self, window_size, gait_engine):
        super().__init__(window_size)
        self.gait_engine = gait_engine_constructor(gait_engine)
        # self.walk_controller = slider_controller.WalkSliderController()
        self.walk_controller = PS_controller.WalkPSController()

        self.current_direction = np.array([0.0, 0.0], dtype=np.float32)
        self.current_speed = 0
        self.target_positions = np.copy(self.pos)

    def set_gait_engine(self, gait_engine: GaitEngine):
        """Set the gait engine to use for generating leg positions."""
        self.gait_engine = gait_engine
        self.target_positions = np.copy(self.pos)

    def update_from_input(self):
        """Update movement parameters from input."""

        inputs = self.walk_controller.get_motion_command()

        if inputs == None:
            return

        self.current_direction = inputs["direction"]
        self.current_speed = inputs["speed"]
        self.rotation_speed = inputs["rotation_speed"]
        fwd_back = inputs["fwd_back"]
        left_right = inputs["left_right"]
        up_down = inputs["up_down"]
        roll = inputs["roll"]
        pitch = inputs["pitch"]
        yaw = inputs["yaw"]

        print("Input command:", inputs)

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
        
    def update_rotation(self):
        """Update the robot's orientation with respect to world."""
        
        if self.rotation_speed == 0:
            return

        angle = np.radians(self.rotation_speed) * consts.DT
        c = np.cos(angle)
        s = np.sin(angle)

        rotation_matrix = np.array([
            [c, -s, 0, 0],
            [s,  c, 0, 0],
            [0,  0, 1, 0],
            [0,  0, 0, 1]
        ], dtype=np.float32)

        theta = np.arctan2(self.base_world[1,0], self.base_world[0,0])
        print(f"Current orientation: {np.degrees(theta):.2f} degrees")

        self.base_world = self.base_world @ rotation_matrix
        self.world_base = np.linalg.inv(self.base_world)

        # Update leg positions to reflect new orientation
        pos_world = base_to_world(leg_to_base(self.pos, self.tm_base_body), self.tm_body_base @ self.world_base)
        pos_base = world_to_base(pos_world, self.tm_base_body @ self.base_world)

        self.pos = base_to_leg(pos_base, self.tm_body_base)

    def update(self):
        """Update per-leg foot positions based on gait + trajectory planning."""

        # update direction with input
        
        self.update_from_input()

        if self.gait_engine.state == State.IDLE and self.current_speed == 0:
            return self.pos

        self.update_rotation()

        # print("Target positions before gait engine:\n", self.target_positions[0:1])
        # print("Current positions before gait engine:\n", self.pos[0:1])

        self.target_positions = base_to_leg(
            world_to_base(
                self.gait_engine.get_step_targets(
                    base_to_world(
                        leg_to_base(self.pos, self.tm_base_body),
                        self.tm_body_base, True),
                    self.current_direction,
                    self.target_positions,
                    (self.target_positions == self.pos).all()),
                self.tm_base_body, True),
            self.tm_body_base)

        # print("Target positions after gait engine:\n", self.target_positions[0:1])

        self.pos = generate_new_positions(
            self.pos,
            self.target_positions,
            speed=self.current_speed if self.gait_engine.state != State.IDLING else consts.IDLING_SPEED,
        )
        # print("Current positions after trajectory planning:\n", self.pos[0:1])
        # import pdb; pdb.set_trace()
        # print("Current positions:\n", self.pos)
        # print("Target positions:\n", self.target_positions)

        return self.pos

        
