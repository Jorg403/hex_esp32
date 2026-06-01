"""
PS5 Gait Demo  —  terminal edition
====================================
A minimal demo that drives the hexapod with a DualSense (PS5) controller
and prints a live status line to the terminal.  No OpenCV window is required,
so this can run on headless hardware as well as on a desktop.

For a full visualisation see src/ps_gait_demo.py.

Usage
-----
    python src/demo_gait.py                    # offline / print mode (no hardware)
    python src/demo_gait.py --IP 192.168.1.42  # Wi-Fi to ESP32
    python src/demo_gait.py --COM COM3         # Bluetooth serial
    python src/demo_gait.py --gait ripple      # start with a different gait

Controls (DualSense / PS5)
--------------------------
  X              Cycle mode: body-pos → tilt → locomotion  (lightbar changes colour)
  Options        (mode 2 only) Cycle gait: tripod → ripple → monopod

  Mode 0  [body position]  — green lightbar
    L-stick X/Y  Shift body left / right and forward / back
    R-stick Y    Raise or lower body height

  Mode 1  [body tilt]  — pink lightbar
    L-stick X    Roll
    L-stick Y    Pitch
    R-stick X    Yaw

  Mode 2  [locomotion]  — blue lightbar
    L-stick      Walk direction + speed
    R-stick X    Rotate in place
    Options      Cycle gait pattern

  Ctrl+C  Quit

Architecture
------------
  GaitPoseGenerator  reads PS5 input, runs the active gait engine, and
                     interpolates all six leg positions each tick.
  LoopHandler        solves inverse kinematics per leg and sends the
                     resulting servo angles to the ESP32 via comm.
"""

import sys
import os
import argparse
import threading
import time

# Ensure the repo root is on sys.path regardless of launch directory.
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from lib.comm.comm_constructor import create_comm
from lib.robotics.pose_generation.gait_pose_generator import GaitPoseGenerator
from lib.robotics.loop_handler import LoopHandler
from lib.robotics.pose_generation.movement.gaits.gait_engine_constructor import GAIT_NAMES


# ── Status display ────────────────────────────────────────────────────────────

_STATUS_HZ    = 4          # terminal refreshes per second
_MODE_NAMES   = ['body-pos', 'tilt', 'locomotion']


def _status_loop(pose_gen: GaitPoseGenerator, stop_event: threading.Event) -> None:
    """Overwrite the current terminal line with live gait / mode / speed info."""
    while not stop_event.is_set():
        engine = pose_gen.gait_engine
        ctrl   = pose_gen.walk_controller

        gait   = pose_gen.current_gait_name
        state  = engine.state.name
        mode   = _MODE_NAMES[getattr(ctrl, 'mode', 0)]
        speed  = pose_gen.current_speed
        rot    = pose_gen.rotation_speed

        # \r keeps the cursor at column 0 so successive writes overwrite the line.
        line = (
            f"\r  gait={gait:<8}  state={state:<10}  "
            f"mode={mode:<12}  spd={speed:.2f}  rot={rot:+.3f}    "
        )
        sys.stdout.write(line)
        sys.stdout.flush()

        time.sleep(1.0 / _STATUS_HZ)


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="PS5 Gait Demo (terminal edition)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--IP',   required=False, help="ESP32 IP address for Wi-Fi")
    parser.add_argument('--COM',  required=False, help="COM port for Bluetooth")
    parser.add_argument(
        '--gait', default='tripod',
        choices=GAIT_NAMES,
        help="Starting gait pattern (default: tripod)",
    )
    args = parser.parse_args()

    # Select communication backend based on the provided argument.
    # Falls back to PrintComm (debug) when neither --IP nor --COM is given.
    if args.IP:
        comm = create_comm('wifi', ip=args.IP)
        conn_label = f"Wi-Fi  {args.IP}"
    elif args.COM:
        comm = create_comm('bluetooth', port=args.COM)
        conn_label = f"Bluetooth  {args.COM}"
    else:
        comm = create_comm('print')
        conn_label = "print (offline — commands are printed, not sent)"

    print(__doc__)
    print(f"  Connection  : {conn_label}")
    print(f"  Start gait  : {args.gait}")
    print(f"  All gaits   : {GAIT_NAMES}")
    print("  Press Ctrl+C to quit.\n")

    # GaitPoseGenerator wires up the PS5 controller, gait engine, and trajectory
    # planner.  window_size is forwarded to the base PoseGenerator but unused here.
    pose_gen     = GaitPoseGenerator(window_size=600, gait_engine=args.gait)
    loop_handler = LoopHandler(pose_gen, comm)

    # Run the control loop in a background daemon thread so the main thread
    # can handle the status display and respond to Ctrl+C cleanly.
    loop_handler.start(threaded=True)

    stop_event = threading.Event()

    # Status display thread — refreshes the terminal line at _STATUS_HZ Hz.
    status_thread = threading.Thread(
        target=_status_loop,
        args=(pose_gen, stop_event),
        daemon=True,
    )
    status_thread.start()

    # Block here until Ctrl+C; daemon threads exit automatically on process end.
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_event.set()
        print("\n\nStopped.")


if __name__ == "__main__":
    main()
