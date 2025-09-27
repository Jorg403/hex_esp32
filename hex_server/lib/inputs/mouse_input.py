import cv2
import threading

class MouseInput:
    def __init__(self, window_name="Control 3D", window_size=500):
        self.window_name = window_name
        self.window_size = window_size
        self.mouse_pressed = False
        self.mouse_pos = [window_size // 2] * 2
        self.lock = threading.Lock()

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self._mouse_callback)

    def _mouse_callback(self, event, x, y, flags, param):
        with self.lock:
            if event == cv2.EVENT_MOUSEMOVE:
                self.mouse_pos = [x, y]
            elif event == cv2.EVENT_LBUTTONDOWN:
                self.mouse_pressed = True
            elif event == cv2.EVENT_LBUTTONUP:
                self.mouse_pressed = False
            elif event == cv2.EVENT_RBUTTONDOWN:
                # Indicar que se presionó botón derecho (manejado desde la lógica)
                self.right_click = True
            else:
                self.right_click = False

    def get_state(self):
        with self.lock:
            return {
                "pos": self.mouse_pos.copy(),
                "pressed": self.mouse_pressed
            }
