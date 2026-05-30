"""
Fluidity Tester
===============
Drive two servos with continuous sine waves to check for mechanical
chatter, missed steps, or jitter.  No controller input is required.

Usage:
    python src/fluidity_tester.py --IP 192.168.x.x
    python src/fluidity_tester.py --COM COM3

Servos 0 and 1 are swept from 0° to 180° and back via func_1.
The loop runs indefinitely; press Ctrl+C to stop.
"""

import argparse
import sys
import os
import time
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from lib.comm.comm_constructor import create_comm


def map_value(value, in_min, in_max, out_min, out_max):
    """Linearly map value from [in_min, in_max] to [out_min, out_max]."""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def func_1(i):
    """Map integer step i (0–360) to a servo angle in [0, 180] via sine."""
    return np.sin(i * 3.14 / 180) * 90 + 90


def main():
    parser = argparse.ArgumentParser(description="Continuous sine-wave servo fluidity test.")
    parser.add_argument('--IP', required=False, help="ESP32 IP address on the local network.")
    parser.add_argument('--COM', required=False, help="COM port for serial communication.")

    args = parser.parse_args()

    if args.IP is not None:
        comm = create_comm('wifi', ip=args.IP)
    elif args.COM is not None:
        comm = create_comm('bluetooth', port=args.COM)
    else:
        print("Error: provide --IP or --COM.")
        return

    print("Connected")

    angle_x = 90
    angle_y = 90
    delta = 0.   # delay between sends (0 = as fast as possible)

    i = 0
    next_angle_x = func_1
    next_angle_y = func_1

    while True:
        time.sleep(delta)
        angle_x = next_angle_x(i)
        comm.send_command(f"set_position 0 {angle_x}")
        time.sleep(delta)
        angle_y = next_angle_y(i)
        comm.send_command(f"set_position 1 {angle_y}")
        i += 1
        if i > 360:
            i = 0

if __name__ == "__main__":
    main()
