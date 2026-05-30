# lib/inputs/ps_input.py
import threading
import time
import numpy as np
from dualsense_controller import DualSenseController, Mapping, UpdateLevel


class PSInput:
    def __init__(self):
        self.lock = threading.Lock()
        self.left_joystick = [0, 0]
        self.right_joystick = [0, 0]
        self.triggers = [0, 0]  # L2, R2
        self.running = False
        self.x_button = False
        self.triangle_button = False
        self.changed = True

        self.controller = self._init_controller()
        # self._start_input_loop()

    def _init_controller(self):
        devices = DualSenseController.enumerate_devices()
        if not devices:
            raise Exception("No PS5 controller found.")

        controller = DualSenseController(
            device_index_or_device_info=devices[0],
            mapping=Mapping.RAW,
            left_joystick_deadzone=10,
            right_joystick_deadzone=10,
            update_level=UpdateLevel.DEFAULT,
        )
        controller.activate()

        # Sticks
        controller.left_stick.on_change(lambda joy: self._on_left_stick(joy.x, joy.y))
        controller.right_stick.on_change(lambda joy: self._on_right_stick(joy.x, joy.y))

        # Triggers
        controller.left_trigger.on_change(lambda t: self._on_trigger(0, t))
        controller.right_trigger.on_change(lambda t: self._on_trigger(1, t))

        # Buttons
        controller.btn_cross.on_down(self._cross_pressed)
        controller.btn_triangle.on_down(self._triangle_pressed)
        return controller
    
    def _cross_pressed(self):
        with self.lock:
            self.x_button = True
            self.changed = True

    def _triangle_pressed(self):
        with self.lock:
            self.triangle_button = True
            self.changed = True

    def _on_left_stick(self, x, y):
        with self.lock:
            self.left_joystick = [x - 127.5, y - 127.5]
            self.changed = True

    def _on_right_stick(self, x, y):
        with self.lock:
            self.right_joystick = [x - 127.5, y - 127.5]
            self.changed = True

    def _on_trigger(self, idx, val):
        with self.lock:
            self.triggers[idx] = val  # [0, 255]
            self.changed = True

    # def _start_input_loop(self):
    #     self.running = True
    #     threading.Thread(target=self._input_loop, daemon=True).start()

    # def _input_loop(self):
    #     while self.running:
    #         self.controller.update()
    #         time.sleep(0.02)  # 50 Hz

    def get_state(self):
        """Devuelve el estado procesado en un dict normalizado [-1, 1]."""
        with self.lock:
            if not self.changed:
                return None
            x_button = self.x_button
            triangle_button = self.triangle_button
            self.x_button = False  # reset after reading
            self.triangle_button = False
            lx, ly = self.left_joystick
            rx, ry = self.right_joystick
            l2, r2 = self.triggers
            self.changed = False

        return {
            "x": x_button,
            "triangle": triangle_button,
            "lx": lx / 127.5,   # [-1, 1]
            "ly": ly / 127.5,
            "rx": rx / 127.5,
            "ry": ry / 127.5,
            "l2": l2 / 255.0,   # [0, 1]
            "r2": r2 / 255.0,
        }

    def stop(self):
        self.running = False
        self.controller.deactivate()
