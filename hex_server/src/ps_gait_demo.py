"""
PS Controller Gait Demo
=======================
Run:  python src/ps_gait_demo.py
      python src/ps_gait_demo.py --IP 192.168.x.x
      python src/ps_gait_demo.py --COM COM3

Controls (PS5 controller):
  X button   — cycle mode: body-pos / tilt / locomotion
  Options    — (mode 2) cycle gait: tripod → ripple → monopod
  Mode 0     — L-stick X/Y = body shift, R-stick Y = height
  Mode 1     — L-stick = roll/pitch, R-stick X = yaw
  Mode 2     — L-stick = walk direction, R-stick X = rotate in place

Press Q in the window to quit.
"""

import sys
import os
import argparse
import threading
import time

import numpy as np
import cv2

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from lib.comm.comm_constructor import create_comm
from lib.robotics.pose_generation.gait_pose_generator import GaitPoseGenerator
from lib.robotics.loop_handler import LoopHandler
from lib.utils.coords_utils import leg_to_base
from lib.robotics.pose_generation.movement.gaits.gait_engine_constructor import GAIT_NAMES
from lib.robotics.pose_generation.movement.gaits.gait_engine import State
import lib.constants as consts

# ──────────────────────────────────────────────────────────────────────────────
# Display constants
# ──────────────────────────────────────────────────────────────────────────────
WIN_W, WIN_H = 900, 620
VIEW_CX, VIEW_CY = 310, 310      # centre of top-down view
SCALE = 13.0                     # px per cm

# Colour palette (BGR)
C_BG        = (30,  30,  30)
C_GRID      = (55,  55,  55)
C_BODY      = (100, 100, 100)
C_HOME      = (80,  80,  80)
C_STANCE    = (60,  200, 60)
C_SWING     = (60,  100, 220)
C_SWING_AIR = (60,  220, 220)
C_TEXT      = (220, 220, 220)
C_HIGHLIGHT = (0,   200, 255)
C_WARN      = (0,   100, 220)
C_GAIT_CLR  = {
    'tripod':  (60,  200, 60),
    'ripple':  (0,   165, 255),
    'monopod': (180, 60,  220),
}

MODE_NAMES = ['Body position', 'Body tilt', 'Locomotion']

CONTROLS = [
    ("X",        "Cycle mode"),
    ("Options",  "Cycle gait  (mode 2)"),
    ("L-stick",  "Mode 0: body XY / Mode 2: walk"),
    ("R-stick Y","Mode 0: height"),
    ("L-stick",  "Mode 1: roll + pitch"),
    ("R-stick X","Mode 1: yaw / Mode 2: rotate"),
    ("Q",        "Quit"),
]


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────
def world_to_px(x_cm, y_cm):
    px = int(VIEW_CX + x_cm * SCALE)
    py = int(VIEW_CY - y_cm * SCALE)   # y-up in robot space → y-down in image
    return px, py


def draw_grid(img):
    spacing = int(5 * SCALE)
    for dx in range(-8, 9):
        x = VIEW_CX + dx * spacing
        cv2.line(img, (x, VIEW_CY - 8*spacing), (x, VIEW_CY + 8*spacing), C_GRID, 1)
    for dy in range(-8, 9):
        y = VIEW_CY + dy * spacing
        cv2.line(img, (VIEW_CX - 8*spacing, y), (VIEW_CX + 8*spacing, y), C_GRID, 1)
    # axes
    cv2.line(img, (VIEW_CX, VIEW_CY - 8*spacing), (VIEW_CX, VIEW_CY + 8*spacing), (70, 70, 70), 1)
    cv2.line(img, (VIEW_CX - 8*spacing, VIEW_CY), (VIEW_CX + 8*spacing, VIEW_CY), (70, 70, 70), 1)


def draw_body_outline(img):
    """Draw a rough hexagon at the leg mount points."""
    mounts = [
        consts.TM_BODY_LEG0[:3, 3],
        consts.TM_BODY_LEG1[:3, 3],
        consts.TM_BODY_LEG2[:3, 3],
        consts.TM_BODY_LEG3[:3, 3],
        consts.TM_BODY_LEG4[:3, 3],
        consts.TM_BODY_LEG5[:3, 3],
    ]
    pts = np.array([world_to_px(m[0], m[1]) for m in mounts], dtype=np.int32)
    cv2.polylines(img, [pts], isClosed=True, color=C_BODY, thickness=2)
    cv2.circle(img, world_to_px(0, 0), 5, C_BODY, -1)
    # forward arrow
    cv2.arrowedLine(img, world_to_px(0, 0), world_to_px(4, 0), C_BODY, 2, tipLength=0.3)


def draw_legs(img, pose_gen):
    engine = pose_gen.gait_engine
    swing_legs = set()
    if engine.state in (State.WALKING, State.UNIDLING):
        swing_legs = set(engine.leg_groups[engine.current_swing_group].tolist())

    # Body-frame leg positions
    pos_body = leg_to_base(pose_gen.pos, pose_gen.tm_base_body)
    home_body = consts.INITIAL_POSITIONS_BODY

    mount_tms = [
        consts.TM_BODY_LEG0, consts.TM_BODY_LEG1, consts.TM_BODY_LEG2,
        consts.TM_BODY_LEG3, consts.TM_BODY_LEG4, consts.TM_BODY_LEG5,
    ]

    for i in range(6):
        mount = mount_tms[i][:3, 3]
        mx, my = world_to_px(mount[0], mount[1])

        fx, fy = pos_body[i, 0], pos_body[i, 1]
        fz     = pos_body[i, 2]
        hx, hy = home_body[i, 0], home_body[i, 1]

        # Home position ring
        cv2.circle(img, world_to_px(hx, hy), 6, C_HOME, 1)

        # Line from mount to foot
        is_swing = i in swing_legs
        is_air   = fz > (consts.GROUND_HEIGHT + 0.5)
        line_col = C_SWING_AIR if (is_swing and is_air) else (C_SWING if is_swing else C_STANCE)
        cv2.line(img, (mx, my), world_to_px(fx, fy), line_col, 1)

        # Foot dot — larger when in air
        radius = 9 if is_air else 7
        cv2.circle(img, world_to_px(fx, fy), radius, line_col, -1)
        cv2.putText(img, str(i), (world_to_px(fx, fy)[0]+10, world_to_px(fx, fy)[1]-4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, C_TEXT, 1)


def draw_info_panel(img, pose_gen):
    engine = pose_gen.gait_engine
    gait   = pose_gen.current_gait_name
    state  = engine.state.name
    phase  = engine.phase_counter
    swing  = engine.current_swing_group

    # Controller mode – read from the walk controller
    ctrl = pose_gen.walk_controller
    mode_name = MODE_NAMES[ctrl.mode] if hasattr(ctrl, 'mode') else '?'

    x0 = WIN_W - 280
    y  = 20
    dy = 22

    def txt(s, col=C_TEXT, scale=0.55, bold=False):
        nonlocal y
        thick = 2 if bold else 1
        cv2.putText(img, s, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, scale, col, thick)
        y += dy

    # ── Gait ──────────────────────────────────────────────────────────────
    gait_col = C_GAIT_CLR.get(gait, C_HIGHLIGHT)
    txt("─── GAIT ───", C_HIGHLIGHT, bold=True)
    txt(f"  {gait.upper()}", gait_col, scale=0.7, bold=True)
    idx = GAIT_NAMES.index(gait) if gait in GAIT_NAMES else 0
    for i, g in enumerate(GAIT_NAMES):
        marker = "► " if i == idx else "  "
        txt(f"  {marker}{g}", gait_col if i == idx else C_HOME)

    y += 6
    txt("─── STATE ──", C_HIGHLIGHT, bold=True)
    state_col = {
        'IDLE': C_HOME, 'UNIDLING': C_WARN,
        'WALKING': C_STANCE, 'IDLING': C_WARN,
    }.get(state, C_TEXT)
    txt(f"  {state}", state_col, scale=0.65, bold=True)
    if engine.state == State.WALKING:
        txt(f"  phase {phase}  swing grp {swing}")
        n_groups = engine._n_groups
        groups_str = "  grps: " + " | ".join(
            ("→" if i == swing else " ") + str(list(engine.leg_groups[i]))
            for i in range(n_groups)
        )
        txt(groups_str, scale=0.38)

    y += 6
    txt("─── MODE ───", C_HIGHLIGHT, bold=True)
    txt(f"  [{ctrl.mode}] {mode_name}", C_SWING)

    y += 6
    txt("─── CONTROLS ───", C_HIGHLIGHT, bold=True)
    for key, desc in CONTROLS:
        cv2.putText(img, f"  {key:<10}", (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, C_HIGHLIGHT, 1)
        cv2.putText(img, desc,           (x0 + 80, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, C_TEXT, 1)
        y += 18

    y += 6
    txt("─── LEGEND ─────────", C_HIGHLIGHT, bold=True)
    for col, label in [(C_STANCE, "Stance"), (C_SWING, "Swing (ground)"), (C_SWING_AIR, "Swing (air)")]:
        cv2.circle(img, (x0 + 8, y - 5), 6, col, -1)
        cv2.putText(img, label, (x0 + 20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, C_TEXT, 1)
        y += 18


def build_frame(pose_gen):
    img = np.full((WIN_H, WIN_W, 3), C_BG, dtype=np.uint8)

    draw_grid(img)
    draw_body_outline(img)
    draw_legs(img, pose_gen)
    draw_info_panel(img, pose_gen)

    # Divider
    cv2.line(img, (WIN_W - 290, 0), (WIN_W - 290, WIN_H), (60, 60, 60), 1)

    # Title
    cv2.putText(img, "Hexapod Gait Demo", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, C_HIGHLIGHT, 2)
    cv2.putText(img, "top-down view  (5 cm grid)", (10, 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, C_HOME, 1)

    return img


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="PS Controller Gait Demo")
    parser.add_argument('--IP',  required=False, help="ESP32 IP for Wi-Fi")
    parser.add_argument('--COM', required=False, help="COM port for Bluetooth")
    args = parser.parse_args()

    if args.IP:
        comm = create_comm('wifi', ip=args.IP)
    elif args.COM:
        comm = create_comm('bluetooth', port=args.COM)
    else:
        comm = create_comm('print')

    pose_gen     = GaitPoseGenerator(window_size=600, gait_engine='tripod')
    loop_handler = LoopHandler(pose_gen, comm)
    loop_handler.start(threaded=True)

    cv2.namedWindow("Hexapod Gait Demo", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Hexapod Gait Demo", WIN_W, WIN_H)

    print("Hexapod Gait Demo running.")
    print(f"Available gaits: {GAIT_NAMES}")
    print("Press Q in the window to quit.\n")

    while True:
        frame = build_frame(pose_gen)
        cv2.imshow("Hexapod Gait Demo", frame)
        key = cv2.waitKey(33) & 0xFF  # ~30 fps
        if key == ord('q') or key == ord('Q') or key == 27:
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
