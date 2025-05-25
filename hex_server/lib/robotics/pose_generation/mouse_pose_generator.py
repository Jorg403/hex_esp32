import cv2
import numpy as np
from lib.robotics.pose_generation.pose_generator import PoseGenerator

class MousePoseGenerator(PoseGenerator):
    def __init__(self, window_size):
        super().__init__(window_size)
        self.mouse_pressed = False
        self.mouse_pos = [window_size // 2] * 2
        self._setup_mouse_callback()

    def _setup_mouse_callback(self):
        cv2.namedWindow("Control 3D")
        cv2.setMouseCallback("Control 3D", self._mouse_callback)

    def _mouse_callback(self, event, x, y, flags, param):
        with self.lock:
            if event == cv2.EVENT_MOUSEMOVE:
                self.mouse_pos = [x, y]
            elif event == cv2.EVENT_LBUTTONDOWN:
                self.mouse_pressed = True
            elif event == cv2.EVENT_LBUTTONUP:
                self.mouse_pressed = False
            elif event == cv2.EVENT_RBUTTONDOWN:
                self.plane = {'xy': 'xz', 'xz': 'yz', 'yz': 'xy'}[self.plane]
                print(f"Plane changed to: {self.plane}")
            elif event == cv2.EVENT_MBUTTONDOWN:
                self.pos[2] += 2
            elif event == cv2.EVENT_MBUTTONUP:
                self.pos[2] -= 2

    def update(self):
        """Update the 3D position based on current mouse input."""
        xlims, ylims, zlims = [0, 21], [-21, 21], [-21, 21]
        mx, my = self.mouse_pos
        with self.lock:
            if self.mouse_pressed:
                if self.plane == 'xy':
                    self.pos[0] = self._map(mx, 0, self.window_size, *xlims)
                    self.pos[1] = self._map(my, 0, self.window_size, *ylims)
                    self.last_win_pos[:2] = mx, my
                elif self.plane == 'xz':
                    self.pos[0] = self._map(mx, 0, self.window_size, *xlims)
                    self.pos[2] = self._map(self.window_size - my, 0, self.window_size, *zlims)
                    self.last_win_pos[0], self.last_win_pos[2] = mx, my
                elif self.plane == 'yz':
                    self.pos[1] = self._map(mx, 0, self.window_size, *ylims)
                    self.pos[2] = self._map(self.window_size - my, 0, self.window_size, *zlims)
                    self.last_win_pos[1], self.last_win_pos[2] = mx, my

    def _map(self, value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
