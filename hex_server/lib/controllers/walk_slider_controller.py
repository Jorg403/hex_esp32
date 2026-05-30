import numpy as np
from lib.controllers.walk_controller import WalkController
from lib.inputs.slider_input import SliderInput

class WalkSliderController(WalkController):
    def __init__(self):
        super().__init__()
        self.sliders = SliderInput(
            n_sliders=9,
            slider_names=[
                "Forward/Backward", "Left/Right", "Up/Down",
                "Roll", "Pitch", "Yaw", "vx", "vy", "rotation_speed"
            ]
        )

    def get_motion_command(self):
        self.sliders.poll()
        sliders = self.sliders.get_state()
        if sliders is None:
            return None

        fwd_back = sliders[0] / 10
        left_right = sliders[1] / 10
        up_down = sliders[2] / 10

        roll = sliders[3] / 100 * 15
        pitch = sliders[4] / 100 * 15
        yaw = sliders[5] / 100 * 15

        vx = sliders[6] / 100
        vy = sliders[7] / 100

        w = sliders[8] / 10

        current_speed = np.linalg.norm([vx, vy])
        if current_speed > 0:
            current_direction = np.array([vx, vy], dtype=np.float32) / current_speed
        else:
            current_direction = np.zeros(2, dtype=np.float32)

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
            "gait":           None,
        }
