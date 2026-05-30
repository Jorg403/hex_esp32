#legacy
import time
import numpy as np
import threading
from lib.robotics.pose_generation.pose_generator import PoseGenerator
from dualsense_controller import DualSenseController, Mapping, UpdateLevel

PLANE_COLORS = {'xy': (0, 0, 255), 'xz': (0, 255, 0), 'yz': (255, 0, 0)}

PLANE_CYCLE = {'xy': 'xz', 'xz': 'yz', 'yz': 'xy'}


class ControllerPoseGenerator(PoseGenerator):
    def __init__(self, window_size):
        super().__init__(window_size)
        self.controller = self._init_controller()
        self.left_joystick = [0, 0]
        self.right_joystick = [0, 0]
        self.old_left_joystick = [0, 0]
        self.old_right_joystick = [0, 0]
        self.leg = 0
        self._start_input_loop()

    def _init_controller(self):
        devices = DualSenseController.enumerate_devices()
        if not devices:
            raise Exception("No PS5 controller found.")

        controller = DualSenseController(
            device_index_or_device_info=devices[0],
            mapping=Mapping.RAW,
            left_joystick_deadzone=5,
            right_joystick_deadzone=5,
            update_level=UpdateLevel.DEFAULT,
        )

        controller.activate()

        # Joystick movement
        controller.left_stick.on_change(lambda joy: self._on_left_stick(joy.x, joy.y))
        controller.right_stick.on_change(lambda joy: self._on_right_stick(joy.x, joy.y))

        # Plane toggle (X / Cross button)
        controller.btn_cross.on_down(self._cycle_plane)
        controller.btn_circle.on_down(self._cycle_leg)

        return controller

    def _cycle_leg(self):
        with self.lock:
            self.leg = (self.leg + 1) % 6
            print(f"Switched to leg: {self.leg}")

    def _cycle_plane(self):
        with self.lock:
            self.plane = PLANE_CYCLE[self.plane]
            print(f"Switched to plane: {self.plane}")
            r, g, b = PLANE_COLORS[self.plane]
            self.controller.lightbar.set_color(r, g, b)

    def _on_left_stick(self, x, y):
        with self.lock:
            self.left_joystick = [x-127.5, y-127.5]

    def _on_right_stick(self, x, y):
        with self.lock:
            self.right_joystick = [x-127.5, y-127.5]

    def _start_input_loop(self):
        self.running = True
        threading.Thread(target=self._input_loop, daemon=True).start()

    def _input_loop(self):
        while self.running:
            self.update()
            time.sleep(0.02)  # 50 Hz

    def update(self):
        """Update the 3D position based on joystick input (relative movement)."""
        with self.lock:
            if self.left_joystick != [0,0] or self.right_joystick != [0,0]:
                dx = self._scale(self.left_joystick[0])
                dy = self._scale(self.left_joystick[1])
                dz = self._scale(self.right_joystick[1])

                # print(f"Joystick: {self.left_joystick}, {self.right_joystick}")

                if self.plane == 'xy':
                    self.pos[self.leg][0] += dx
                    self.pos[self.leg][1] += dy
                    self.pos[self.leg][2] += -dz
                elif self.plane == 'xz':
                    self.pos[self.leg][0] += dx
                    self.pos[self.leg][1] += -dz
                    self.pos[self.leg][2] += -dy
                elif self.plane == 'yz':
                    self.pos[self.leg][0] += -dz
                    self.pos[self.leg][1] += dx
                    self.pos[self.leg][2] += -dy

                self.pos[self.leg] = [
                    max(min(p, 21), -21) for p in self.pos[self.leg]
                ]
                # self.pos[0] = max(self.pos[0],0)

                self.old_left_joystick = self.left_joystick.copy()
                self.old_right_joystick = self.right_joystick.copy()

                self.last_win_pos = self.window_size*(np.array([0,0.5,0.5])+self.pos[self.leg]/np.array([21,42,-42]))
        return self.pos

    def _scale(self, val):
        """Scale joystick value to movement increment."""
        return val / 500.0

    def stop(self):
        self.running = False
        self.controller.deactivate()
