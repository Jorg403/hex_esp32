import argparse
import requests
import cv2
import time
import threading
import numpy as np

import sys
import os

# Adjust sys.path for parent directory
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from lib.comm.comm_constructor import create_comm
import lib.constants as consts

def map_value(value, in_min, in_max, out_min, out_max):
    """Mapea un valor de un rango a otro."""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def main():
    parser = argparse.ArgumentParser(description="Control de servos con ratón (posición).")
    parser.add_argument('--IP', required=False, help="La dirección IP del ESP32 en la red local.")
    parser.add_argument('--COM', required=False, help="Puerto COM para la comunicación serial.")
    parser.add_argument('--size', type=int, default=500, help="Tamaño de la ventana cuadrada (px).")
    
    args = parser.parse_args()

    if args.IP is not None:
        comm = create_comm('wifi', ip=args.IP)
    elif args.COM is not None:
        comm = create_comm('bluetooth', port=args.COM)
    else:
        comm = create_comm('print')

    window_size = args.size

    print(f"Conectado")
    print(f"Ventana de control: {window_size}x{window_size}")

    mouse_x, mouse_y = window_size // 2, window_size // 2
    inside_window = False
    mouse_pressed_L = False
    mouse_pressed_R = False
    last_angle_x_pressed = window_size // 2
    last_angle_x_released = window_size // 2
    lock = threading.Lock()

    def mouse_callback(event, x, y, flags, param):
        nonlocal mouse_x, mouse_y, inside_window, mouse_pressed_L, mouse_pressed_R
        if event == cv2.EVENT_MOUSEMOVE:
            with lock:
                mouse_x, mouse_y = x, y
                inside_window = (0 <= x < window_size) and (0 <= y < window_size)
        elif event == cv2.EVENT_LBUTTONDOWN:
            with lock:
                mouse_pressed_L = True
        elif event == cv2.EVENT_LBUTTONUP:
            with lock:
                mouse_pressed_L = False
        elif event == cv2.EVENT_RBUTTONDOWN:
            with lock:
                mouse_pressed_R = True
        elif event == cv2.EVENT_RBUTTONUP:
            with lock:
                mouse_pressed_R = False

    cv2.namedWindow("Control de Servos")
    cv2.setMouseCallback("Control de Servos", mouse_callback)

    def send_loop():
        nonlocal last_angle_x_pressed, last_angle_x_released
        old_mouse_x = 0
        old_mouse_y = 0
        delta = 0.01
        while True:
            time.sleep(delta)
            with lock:
                if inside_window and (mouse_x != old_mouse_x or mouse_y != old_mouse_y):
                    old_mouse_x = mouse_x
                    old_mouse_y = mouse_y
                    
                    if mouse_pressed_L:
                        angle_x = map_value(mouse_x, 0, window_size, consts.SERVO_MIN_POS[0], consts.SERVO_MAX_POS[0])
                        comm.send_command(f"set_position 0 {angle_x}")
                        last_angle_x_pressed = int(mouse_x)
                    elif mouse_pressed_R:
                        angle_x = map_value(window_size-mouse_x, 0, window_size, consts.SERVO_MIN_POS[2], consts.SERVO_MAX_POS[2])
                        comm.send_command(f"set_position 2 {angle_x}")
                        last_angle_x_released = int(mouse_x)
                    time.sleep(delta)

                    angle_y = map_value(window_size-mouse_y, 0, window_size, consts.SERVO_MIN_POS[1], consts.SERVO_MAX_POS[1])
                    comm.send_command(f"set_position 1 {angle_y}")

    sender_thread = threading.Thread(target=send_loop, daemon=True)
    sender_thread.start()

    while True:
        # Crear fondo negro
        frame = np.zeros((window_size, window_size, 3), dtype=np.uint8)

        with lock:
            current_mouse_y = mouse_y

        # Dibujar línea verde (clic izquierdo)
        cv2.line(frame, (last_angle_x_pressed, 0), (last_angle_x_pressed, window_size), (0, 255, 0), 2)

        # Dibujar línea naranja (clic derecho)
        cv2.line(frame, (last_angle_x_released, 0), (last_angle_x_released, window_size), (0, 165, 255), 2)

        # Dibujar línea roja (mouse_y actual)
        cv2.line(frame, (0, current_mouse_y), (window_size, current_mouse_y), (0, 0, 255), 2)

        cv2.imshow("Control de Servos", frame)

        key = cv2.waitKey(30) & 0xFF
        if key == 27:  # Escape
            print("Saliendo...")
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
