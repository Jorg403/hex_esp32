import numpy as np
from lib.controllers.walk_controller import WalkController
from lib.inputs.ps_input import PSInput
from lib.robotics.pose_generation.movement.gaits.gait_engine_constructor import GAIT_NAMES

MODE_COLORS = [
    (0,   255, 0),    # mode 0 – body position
    (255, 0,   255),  # mode 1 – tilt/camera
    (0,   0,   255),  # mode 2 – locomotion
]


class WalkPSController(WalkController):
    def __init__(self):
        super().__init__()
        self.ps = PSInput()
        self.mode = 0
        self.gait_idx = 0
        self._prev_x = False
        self._prev_triangle = False
        self.ps.controller.lightbar.set_color(*MODE_COLORS[self.mode])

    def get_motion_command(self):
        state = self.ps.get_state()
        if state is None:
            return None

        # Debounced mode cycle (X button)
        x_pressed = bool(state["x"])
        if x_pressed and not self._prev_x:
            self.mode = (self.mode + 1) % len(MODE_COLORS)
            self.ps.controller.lightbar.set_color(*MODE_COLORS[self.mode])
        self._prev_x = x_pressed

        # Debounced gait cycle (triangle button, locomotion mode only)
        triangle_pressed = bool(state.get("triangle", False))
        gait_changed = False
        if self.mode == 2 and triangle_pressed and not self._prev_triangle:
            self.gait_idx = (self.gait_idx + 1) % len(GAIT_NAMES)
            gait_changed = True
        self._prev_triangle = triangle_pressed

        # --- map sticks per mode ---
        if self.mode == 0:
            fwd_back   = state["ly"] * 5
            left_right = state["lx"] * 5
            up_down    = state["ry"] * 7
            roll = pitch = yaw = vx = vy = w = 0.0

        elif self.mode == 1:
            fwd_back = left_right = up_down = vx = vy = w = 0.0
            roll  = -state["lx"] * 6.0
            pitch =  state["ly"] * 6.0
            yaw   =  state["rx"] * 20.0

        else:  # mode 2 – locomotion
            fwd_back = left_right = up_down = roll = pitch = yaw = 0.0
            vx = -state["ly"] / 5.0
            vy = -state["lx"] / 5.0
            w  =  -state["rx"] / 40.0   # rotation speed (±0.1 at stick limit)

        current_speed = np.linalg.norm([vx, vy])
        current_direction = (
            np.array([vx, vy], dtype=np.float32) / current_speed
            if current_speed > 0 else np.zeros(2, dtype=np.float32)
        )

        return {
            "direction":      current_direction,
            "speed":          min(current_speed, 1.0),
            "fwd_back":       fwd_back,
            "left_right":     left_right,
            "up_down":        up_down,
            "roll":           roll,
            "pitch":          pitch,
            "yaw":            yaw,
            "rotation_speed": w,
            # Non-None only on the frame the button was pressed
            "gait": GAIT_NAMES[self.gait_idx] if gait_changed else None,
        }
