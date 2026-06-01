"""
Hexapod WiFi + PS5 gait demo
Usage:  python src/demo.py --IP 192.168.x.x

PS5 controls
────────────
  X               cycle mode  (green / pink / blue lightbar)
  ── Mode 0  [body position] ──────────────────────────────
  L-stick X/Y     shift body left/right and forward/back
  R-stick Y       body height
  ── Mode 1  [body tilt] ──────────────────────────────────
  L-stick X       roll
  L-stick Y       pitch
  R-stick X       yaw
  ── Mode 2  [locomotion] ─────────────────────────────────
  L-stick         walk direction
  R-stick X       rotate in place
  Options         cycle gait: tripod → ripple → monopod
"""

import sys
import os
import argparse

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from lib.comm.comm_constructor import create_comm
from lib.robotics.pose_generation.gait_pose_generator import GaitPoseGenerator
from lib.robotics.loop_handler import LoopHandler
from lib.robotics.pose_generation.movement.gaits.gait_engine_constructor import GAIT_NAMES


def main():
    parser = argparse.ArgumentParser(description="Hexapod WiFi + PS5 gait demo")
    parser.add_argument('--IP', required=True, help="ESP32 IP address (e.g. 192.168.1.42)")
    args = parser.parse_args()

    print(__doc__)
    print(f"Available gaits : {GAIT_NAMES}")
    print(f"Connecting to   : {args.IP}")
    print("─" * 48)

    comm     = create_comm('wifi', ip=args.IP)
    pose_gen = GaitPoseGenerator(window_size=500, gait_engine='tripod')

    loop_handler = LoopHandler(pose_gen, comm)
    loop_handler.start(threaded=False)   # blocks until Ctrl+C


if __name__ == "__main__":
    main()
