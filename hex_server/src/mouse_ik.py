import argparse
import cv2
import time
import threading
import numpy as np
import sys
import os

# Adjust sys.path for parent directory
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from lib.comm.comm_constructor import create_comm
from lib.robotics.inverse_kinematics import IKSolver
import lib.constants as consts

def main():
    parser = argparse.ArgumentParser(description="Control de servos con ratón (posición 3D).")
    parser.add_argument('--IP', required=False, help="La dirección IP del ESP32 en la red local.")
    parser.add_argument('--COM', required=False, help="Puerto COM para la comunicación serial.")
    parser.add_argument('--size', type=int, default=500, help="Tamaño de la ventana cuadrada (px).")
    
    args = parser.parse_args()

    if args.IP:
        comm = create_comm('wifi', ip=args.IP)
    elif args.COM:
        comm = create_comm('bluetooth', port=args.COM)
    else:
        comm = create_comm('print')

    window_size = args.size
    mouse_x, mouse_y = window_size // 2, window_size // 2
    plane = 'xy'
    goal_reached = True
    mouse_pressed_L = False
    lock = threading.Lock()

    # 3D coordinates
    pos = np.array([10.0, 0.0, 6.0], dtype=np.float32)
    ik = IKSolver(pos)
    # last_x, last_y, last_z = window_size // 2, window_size // 2, window_size // 2
    last_win_pos = [window_size // 2, window_size // 2, window_size // 2]

    def map_value(value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    travel_start_t = 0.0

    def update_pos():
        nonlocal pos, ik, goal_reached, travel_start_t
        thetas, reached = ik.calculate_joint_angles_dt(pos)
        if thetas[0] is not None and thetas[1] is not None and thetas[2] is not None:
            comm.send_command(f"set_position 0 {thetas[0]}")
            time.sleep(consts.DT/4)
            comm.send_command(f"set_position 1 {thetas[1]}")
            time.sleep(consts.DT/4)
            comm.send_command(f"set_position 2 {thetas[2]}")
        if goal_reached and not reached:
            travel_start_t = time.time()
        elif not goal_reached and reached:
            travel_time = time.time() - travel_start_t
            print(f"Travel time: {travel_time:.2f} seconds")
        goal_reached = reached

    def mouse_callback(event, x, y, flags, param):
        nonlocal mouse_x, mouse_y, mouse_pressed_L, plane
        if event == cv2.EVENT_MOUSEMOVE:
            with lock:
                mouse_x, mouse_y = x, y
        elif event == cv2.EVENT_LBUTTONDOWN:
            with lock:
                mouse_pressed_L = True
        elif event == cv2.EVENT_LBUTTONUP:
            with lock:
                mouse_pressed_L = False
        elif event == cv2.EVENT_RBUTTONDOWN:
            with lock:
                # Cycle through planes: xy → xz → yz → xy
                plane = {'xy': 'xz', 'xz': 'yz', 'yz': 'xy'}[plane]
                print(f"Plane changed to: {plane}")
        elif event == cv2.EVENT_MBUTTONDOWN:
            with lock:
                pos[2] += 2
                update_pos()
                print(f"Z position changed to: {pos[2]}")
        elif event == cv2.EVENT_MBUTTONUP:
            with lock:
                pos[2] -= 2
                update_pos()
                print(f"Z position changed to: {pos[2]}")

    cv2.namedWindow("Control 3D")
    cv2.setMouseCallback("Control 3D", mouse_callback)

    def send_loop():
        nonlocal mouse_x, mouse_y, mouse_pressed_L, pos, last_win_pos
        xlims = [0, 21]
        ylims = [-21, 21]
        zlims = [-21, 21]
        while True:
            start_time = time.time()
            with lock:
                if mouse_pressed_L:
                    
                    if plane == 'xy':
                        pos[0] = map_value(mouse_x, 0, window_size, xlims[0], xlims[1])
                        pos[1] = map_value(mouse_y, 0, window_size, ylims[0], ylims[1])
                        last_win_pos[0] = mouse_x
                        last_win_pos[1] = mouse_y
                    elif plane == 'xz':
                        pos[0] = map_value(mouse_x, 0, window_size, xlims[0], xlims[1])
                        pos[2] = map_value(window_size - mouse_y, 0, window_size, zlims[0], zlims[1])
                        last_win_pos[0] = mouse_x
                        last_win_pos[2] = mouse_y
                    elif plane == 'yz':
                        pos[1] = map_value(mouse_x, 0, window_size, ylims[0], ylims[1])
                        pos[2] = map_value(window_size - mouse_y, 0, window_size, zlims[0], zlims[1])
                        last_win_pos[1] = mouse_x
                        last_win_pos[2] = mouse_y
                    
                    update_pos()
                elif not goal_reached:
                    update_pos()
                else:
                    time.sleep(consts.DT/2)
            
            elapsed_time = time.time() - start_time
            if elapsed_time < consts.DT:
                time.sleep(consts.DT - elapsed_time)

    sender_thread = threading.Thread(target=send_loop, daemon=True)
    sender_thread.start()

    while True:
        frame = np.zeros((window_size, window_size, 3), dtype=np.uint8)
        color = {'xy': (255, 0, 0), 'xz': (0, 255, 0), 'yz': (0, 0, 255)}

        with lock:

            # Draw crosshair at last mouse position depending on plane
            if plane == 'xy':
                cv2.drawMarker(frame, (last_win_pos[0], last_win_pos[1]), color[plane], markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
                cv2.line(frame, (0, window_size // 2), (last_win_pos[0], last_win_pos[1]), color[plane], 1)
            elif plane == 'xz':
                cv2.drawMarker(frame, (last_win_pos[0], last_win_pos[2]), color[plane], markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
                cv2.line(frame, (0, window_size // 2), (last_win_pos[0] // 2, last_win_pos[2] // 2), color[plane], 1)
                cv2.line(frame, (last_win_pos[0] // 2, last_win_pos[2] // 2), (last_win_pos[0], last_win_pos[2]), color[plane], 1)
            elif plane == 'yz':
                cv2.drawMarker(frame, (last_win_pos[1], last_win_pos[2]), color[plane], markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
                cv2.line(frame, (window_size // 2, window_size // 2), ((last_win_pos[1]+window_size//2)//2, last_win_pos[2] // 2), color[plane], 1)
                cv2.line(frame, ((last_win_pos[1]+window_size//2)//2, last_win_pos[2] // 2), (last_win_pos[1], last_win_pos[2]), color[plane], 1)

            
            # Show current 3D position
            text = f"x: {pos[0]:.2f}, y: {pos[1]:.2f}, z: {pos[2]:.2f}, plane: {plane}"
            cv2.putText(frame, text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow("Control 3D", frame)
        key = cv2.waitKey(30) & 0xFF
        if key == 27:
            print("Saliendo...")
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
