# lib/controllers/walk_ps_controller.py
import numpy as np
from lib.controllers.walk_controller import WalkController
from lib.inputs.ps_input import PSInput

MODE_COLORS = [(0, 255, 0), (255, 0, 255), (0, 0, 255)]

class WalkPSController(WalkController):
    def __init__(self):
        super().__init__()
        self.ps = PSInput()
        self.mode = 0
        self.ps.controller.lightbar.set_color(*MODE_COLORS[self.mode])

    def get_motion_command(self):
        state = self.ps.get_state()
        if state is None:
            return None

        if state["x"]:
            self.mode = (self.mode+1) % 3
            self.ps.controller.lightbar.set_color(*MODE_COLORS[self.mode])

        if self.mode == 0:
            # Mapear sticks
            fwd_back = state["ly"]*5        # stick izq arriba/abajo
            left_right = state["lx"]*5       # stick izq izq/dcha
            up_down = state["ry"]*7          # stick derecho arriba/abajo

            roll = 0.0 
            pitch = 0.0
            yaw = 0.0  

            # Velocidades de traslación en XY
            vx = 0.0
            vy = 0.0

            w = 0.0
        elif self.mode == 1:
            # Modo de control de cámara
            fwd_back = 0.0
            left_right = 0.0
            up_down = 0.0

            roll = -state["lx"] * 6.0     # stick derecho izq/dcha
            pitch = state["ly"] * 6.0   # stick derecho arriba/abajo
            yaw = state["rx"] * 20.0     # stick izq izq/dcha

            vx = 0.0
            vy = 0.0

            w = 0.0
        else:
            # Mapear sticks
            fwd_back = 0.0
            left_right = 0.0
            up_down = 0.0

            roll = 0.0
            pitch = 0.0
            yaw = 0.0

            # Velocidades de traslación en XY
            vx = -state["ly"]/5.0
            vy = -state["lx"]/5.0
            w = state["rx"]/10.0

        current_speed = np.linalg.norm([vx, vy])
        if current_speed > 0:
            current_direction = np.array([vx, vy], dtype=np.float32) / current_speed
        else:
            current_direction = np.zeros(2, dtype=np.float32)

        return {
            "direction": current_direction,
            "speed": min(current_speed, 1.0),
            "fwd_back": fwd_back,
            "left_right": left_right,
            "up_down": up_down,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "rotation_speed": w
        }
