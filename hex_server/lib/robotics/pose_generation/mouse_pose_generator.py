import numpy as np
from lib.robotics.pose_generation.pose_generator import PoseGenerator
from lib.inputs.mouse_input import MouseInput

class MousePoseGenerator(PoseGenerator):
    def __init__(self, window_size):
        super().__init__(window_size)
        self.mouse = MouseInput(window_size=window_size)

    def update(self):
        """Update the 3D position based on current mouse input."""
        xlims, ylims, zlims = [0, 21], [-21, 21], [-21, 21]
        state = self.mouse.get_state()
        mx, my = state["pos"]

        with self.lock:
            if state["pressed"]:
                pos_i = self.pos[0]
                if self.plane == 'xy':
                    pos_i[0] = self._map(mx, 0, self.window_size, *xlims)
                    pos_i[1] = self._map(my, 0, self.window_size, *ylims)
                    self.last_win_pos[:2] = mx, my
                elif self.plane == 'xz':
                    pos_i[0] = self._map(mx, 0, self.window_size, *xlims)
                    pos_i[2] = self._map(self.window_size - my, 0, self.window_size, *zlims)
                    self.last_win_pos[0], self.last_win_pos[2] = mx, my
                elif self.plane == 'yz':
                    pos_i[1] = self._map(mx, 0, self.window_size, *ylims)
                    pos_i[2] = self._map(self.window_size - my, 0, self.window_size, *zlims)
                    self.last_win_pos[1], self.last_win_pos[2] = mx, my

                # replicar para todas las patas
                for i in range(1, len(self.pos)):
                    self.pos[i] = pos_i.copy()

    def _map(self, value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
