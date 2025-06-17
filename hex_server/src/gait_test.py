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
    pose_gen = create_pose_generator('tripod_gait', window_size=args.size)
    loop_handler = LoopHandler(pose_gen, comm)
    loop_handler.start(threaded=False)

if __name__ == "__main__":
    main()
