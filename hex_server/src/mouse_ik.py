import argparse
import numpy as np
import cv2
import sys
import os
import threading
import time

# Ensure the parent directory is in the path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from lib.comm.comm_constructor import create_comm
from lib.robotics.inverse_kinematics import IKSolver
import lib.constants as consts
from lib.robotics.pose_generation.pose_generator_constructor import create_pose_generator
from lib.robotics.loop_handler import LoopHandler

def main():
    parser = argparse.ArgumentParser(description="3D Position Control Interface")
    parser.add_argument('--IP', required=False, help="IP address for Wi-Fi connection to ESP32")
    parser.add_argument('--COM', required=False, help="COM port for Bluetooth communication")
    parser.add_argument('--size', type=int, default=800, help="Size of the square window (px)")
    args = parser.parse_args()

    # Communication setup
    if args.IP:
        comm = create_comm('wifi', ip=args.IP)
    elif args.COM:
        comm = create_comm('bluetooth', port=args.COM)
    else:
        comm = create_comm('print')

    # Initialize control components
    pose_gen = create_pose_generator('controller', window_size=args.size)
    loop_handler = LoopHandler(pose_gen, comm)
    loop_handler.start()

    # Main render loop
    while True:
        frame = np.zeros((args.size, args.size, 3), dtype=np.uint8)
        with pose_gen.lock:
            pos = pose_gen.pos[pose_gen.leg].copy()
            plane = pose_gen.plane
            last = np.round(pose_gen.last_win_pos.copy()).astype(int)

            color = {'xy': (255, 0, 0), 'xz': (0, 255, 0), 'yz': (0, 0, 255)}[plane]

            if plane == 'xy':
                cv2.drawMarker(frame, (last[0], last[1]), color, markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
                cv2.line(frame, (0, args.size // 2), (last[0], last[1]), color, 1)
            elif plane == 'xz':
                cv2.drawMarker(frame, (last[0], last[2]), color, markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
                cv2.line(frame, (0, args.size // 2), (last[0] // 2, last[2] // 2), color, 1)
                cv2.line(frame, (last[0] // 2, last[2] // 2), (last[0], last[2]), color, 1)
            elif plane == 'yz':
                cv2.drawMarker(frame, (last[1], last[2]), color, markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
                cv2.line(frame, (args.size // 2, args.size // 2), ((last[1] + args.size // 2) // 2, last[2] // 2), color, 1)
                cv2.line(frame, ((last[1] + args.size // 2) // 2, last[2] // 2), (last[1], last[2]), color, 1)

            text = f"x: {pos[0]:.2f}, y: {pos[1]:.2f}, z: {pos[2]:.2f}, plane: {plane}"
            cv2.putText(frame, text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow("Control 3D", frame)
        if cv2.waitKey(30) & 0xFF == 27:
            print("Exiting...")
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
