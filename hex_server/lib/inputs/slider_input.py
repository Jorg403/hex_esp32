import cv2
import threading

class SliderInput:
    def __init__(self, n_sliders=3, window_name="Control Sliders", slider_names=None):
        self.n_sliders = n_sliders
        self.window_name = window_name
        self.values = [0] * n_sliders
        self.lock = threading.Lock()
        self.last_values = self.values.copy()

        cv2.namedWindow(self.window_name)

        # Crear sliders
        for i in range(n_sliders):
            name = slider_names[i] if slider_names and i < len(slider_names) else f"Slider {i+1}"
            cv2.createTrackbar(name, self.window_name, 100, 200, self._on_change(i))

    def _on_change(self, index):
        def callback(val):
            with self.lock:
                self.values[index] = val - 100  # [0,200] -> [-100,100]
        return callback

    def get_state(self):
        with self.lock:
            if self.values != self.last_values:
                self.last_values = self.values.copy()
                return self.values.copy()
            return None

    def poll(self):
        # Necesario para mantener viva la ventana
        cv2.waitKey(1)
