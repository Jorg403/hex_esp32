"""
Hexapod Simulation Server
==========================
Behaves like the ESP32 firmware over HTTP.  Start this first, then point
any demo script at --IP 127.0.0.1:<port> to test without hardware.

The server:
  1. Listens for  GET /?cmd=set_positions a0,...,a17
     (the exact URL that WifiComm sends)
  2. Solves forward kinematics → recovers 6 leg positions in body frame
  3. Renders a live OpenCV window:
       Left   — top-down body-frame view with stance/swing colour coding
       Right  — server status, per-leg Z height bars, inferred gait groups

Usage:
    python src/sim_server.py                    # port 8080, all interfaces
    python src/sim_server.py --port 9090
    python src/sim_server.py --host 0.0.0.0 --port 80

Then in a separate terminal, target any demo at this server:
    python src/demo_gait.py    --IP 127.0.0.1:8080
    python src/ps_gait_demo.py --IP 127.0.0.1:8080

Press Q in the window to quit.
"""

import sys
import os
import argparse
import threading
import time
import math
import socket
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

import numpy as np
import cv2

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import lib.constants as consts
from lib.utils.coords_utils import leg_to_base


# ── Forward Kinematics ─────────────────────────────────────────────────────────

def fk_leg(theta1_deg: float, theta2_deg: float, theta3_deg: float) -> np.ndarray:
    """
    Inverse of the IK in lib/robotics/inverse_kinematics.py.
    Returns (x, y, z) in leg frame given three joint angles in degrees.

    Derivation
    ----------
    IK convention:
      phi2 = theta2 + π/2   (L2 elevation angle from horizontal in sagittal plane)
      phi3 = theta2 − theta3 (L3 elevation angle; see IK law-of-cosines geometry)

    Then:
      s  = L2·cos(phi2) + L3·cos(phi3)   horizontal reach past the L1 base
      z  = L2·sin(phi2) + L3·sin(phi3)   height
      r  = s + L1                         total reach from leg origin
      x  =  r·cos(theta1)
      y  = −r·sin(theta1)                 IK negated y on input, so FK negates back
    """
    L1, L2, L3 = consts.LEG_LENGTHS
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    t3 = math.radians(theta3_deg)

    phi2 = t2 + math.pi / 2
    phi3 = t2 - t3

    s = L2 * math.cos(phi2) + L3 * math.cos(phi3)
    z = L2 * math.sin(phi2) + L3 * math.sin(phi3)
    r = s + L1

    x =  r * math.cos(t1)
    y = -r * math.sin(t1)

    return np.array([x, y, z], dtype=np.float32)


# ── Shared simulation state ────────────────────────────────────────────────────

class SimState:
    """Thread-safe container updated by the HTTP handler, read by the renderer."""

    def __init__(self):
        self.pos_body  = consts.INITIAL_POSITIONS_BODY.copy()  # (6, 3) body frame
        self.pos_leg   = consts.INITIAL_POSITIONS.copy()       # (6, 3) leg frame
        self.cmd_count = 0
        self.last_t    = time.time()
        self._prev_t   = time.time()
        self.rate_hz   = 0.0
        self.lock      = threading.Lock()

    def update(self, angles_flat: list) -> None:
        """Apply FK to 18 joint angles and update body-frame leg positions."""
        pos_leg = np.zeros((6, 3), dtype=np.float32)
        for i in range(6):
            pos_leg[i] = fk_leg(angles_flat[i*3], angles_flat[i*3+1], angles_flat[i*3+2])

        # Transform leg-frame positions to body frame (no extra body tilt here —
        # the angles already encode whatever tilt the controller applied).
        pos_body = leg_to_base(pos_leg, np.eye(4, dtype=np.float32))

        now = time.time()
        with self.lock:
            self.pos_leg  = pos_leg
            self.pos_body = pos_body
            self.cmd_count += 1
            dt = now - self._prev_t
            # Exponential moving average smooths the rate display.
            instant = 1.0 / dt if dt > 1e-6 else 0.0
            self.rate_hz = 0.85 * self.rate_hz + 0.15 * instant
            self._prev_t = now
            self.last_t  = now


# ── HTTP handler ───────────────────────────────────────────────────────────────

def make_handler(state: SimState):
    """Return an HTTP handler class that shares state with the renderer."""

    class HexHandler(BaseHTTPRequestHandler):
        """
        Minimal ESP32 firmware emulator.
        Accepts:  GET /?cmd=set_positions a0,a1,...,a17
        Returns:  200 OK  (body "OK")
        Any other command is silently accepted (allows future extensions).
        """

        def do_GET(self):
            parsed = urlparse(self.path)
            params = parse_qs(parsed.query)
            cmd_list = params.get('cmd', [])
            cmd = cmd_list[0] if cmd_list else ''

            if cmd.startswith('set_positions'):
                angles_str = cmd[len('set_positions'):].strip()
                try:
                    angles = [float(a) for a in angles_str.split(',')]
                    if len(angles) == 18:
                        state.update(angles)
                except ValueError:
                    pass  # malformed payload — ignore

            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'')

        def log_message(self, fmt, *args):
            pass  # suppress noisy default access log

    return HexHandler


# ── Rendering constants ────────────────────────────────────────────────────────

WIN_W, WIN_H    = 1020, 660
VIEW_CX         = 310          # top-down view centre (px)
VIEW_CY         = 310
SCALE           = 13.0         # px per cm
DIVIDER_X       = 640          # left / right panel boundary

# Colours (BGR)
C_BG        = (30,  30,  30)
C_GRID      = (55,  55,  55)
C_AXES      = (70,  70,  70)
C_BODY      = (100, 100, 100)
C_HOME      = (65,  65,  65)
C_STANCE    = (60,  200, 60)   # foot on ground
C_SWING     = (60,  220, 220)  # foot in air
C_TEXT      = (220, 220, 220)
C_DIM       = (120, 120, 120)
C_HIGHLIGHT = (0,   200, 255)
C_WARN      = (40,  100, 240)  # stale / warning
C_BAR_BG    = (55,  55,  55)
C_GROUND    = (160, 160, 60)   # ground-level marker on height bars

# A foot is considered airborne when its body-frame Z is above this threshold.
SWING_THRESHOLD = consts.GROUND_HEIGHT + 0.5

# Height bar Z range (cm)
Z_BAR_MIN = consts.GROUND_HEIGHT - 1.0
Z_BAR_MAX = consts.GROUND_HEIGHT + 5.0
Z_BAR_RANGE = Z_BAR_MAX - Z_BAR_MIN

# Leg gait-group colours for the inferred-groups panel (cycles through groups 0/1/2).
_GROUP_COLS = [(60, 200, 60), (60, 220, 220), (200, 100, 220)]


# ── Drawing helpers ────────────────────────────────────────────────────────────

def to_px(x_cm: float, y_cm: float):
    """Convert body-frame (cm) to pixel coords (top-down view)."""
    return int(VIEW_CX + x_cm * SCALE), int(VIEW_CY - y_cm * SCALE)


def draw_grid(img: np.ndarray) -> None:
    """5 cm background grid with highlighted X/Y axes."""
    sp = int(5 * SCALE)
    for d in range(-8, 9):
        xp = VIEW_CX + d * sp
        cv2.line(img, (xp, VIEW_CY - 8*sp), (xp, VIEW_CY + 8*sp), C_GRID, 1)
        yp = VIEW_CY + d * sp
        cv2.line(img, (VIEW_CX - 8*sp, yp), (VIEW_CX + 8*sp, yp), C_GRID, 1)
    cv2.line(img, (VIEW_CX, VIEW_CY - 8*sp), (VIEW_CX, VIEW_CY + 8*sp), C_AXES, 1)
    cv2.line(img, (VIEW_CX - 8*sp, VIEW_CY), (VIEW_CX + 8*sp, VIEW_CY), C_AXES, 1)


def draw_body_outline(img: np.ndarray) -> None:
    """Body hexagon (leg mount points) with forward-direction arrow."""
    tms = [consts.TM_BODY_LEG0, consts.TM_BODY_LEG1, consts.TM_BODY_LEG2,
           consts.TM_BODY_LEG3, consts.TM_BODY_LEG4, consts.TM_BODY_LEG5]
    pts = np.array([to_px(tm[0, 3], tm[1, 3]) for tm in tms], dtype=np.int32)
    cv2.polylines(img, [pts], isClosed=True, color=C_BODY, thickness=2)
    cv2.circle(img, to_px(0, 0), 5, C_BODY, -1)
    cv2.arrowedLine(img, to_px(0, 0), to_px(4, 0), C_BODY, 2, tipLength=0.3)


def draw_legs(img: np.ndarray, pos_body: np.ndarray) -> None:
    """Leg lines and foot dots; cyan = swing (air), green = stance (ground)."""
    tms = [consts.TM_BODY_LEG0, consts.TM_BODY_LEG1, consts.TM_BODY_LEG2,
           consts.TM_BODY_LEG3, consts.TM_BODY_LEG4, consts.TM_BODY_LEG5]

    for i in range(6):
        mx, my = int(tms[i][0, 3] * SCALE + VIEW_CX), int(VIEW_CY - tms[i][1, 3] * SCALE)
        fx, fy, fz = pos_body[i]
        hx, hy = consts.INITIAL_POSITIONS_BODY[i, :2]

        in_air = fz > SWING_THRESHOLD
        col    = C_SWING if in_air else C_STANCE

        # Home ring
        cv2.circle(img, to_px(hx, hy), 6, C_HOME, 1)

        # Mount → foot line
        cv2.line(img, (mx, my), to_px(fx, fy), col, 1)

        # Foot dot (larger when elevated)
        radius = 9 if in_air else 7
        cv2.circle(img, to_px(fx, fy), radius, col, -1)

        # Leg index label
        lp = to_px(fx, fy)
        cv2.putText(img, str(i), (lp[0] + 10, lp[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, C_TEXT, 1)


def draw_height_bars(img: np.ndarray, pos_body: np.ndarray,
                     x0: int, y0: int, bar_w: int = 200) -> int:
    """
    Horizontal Z-height bars for each leg.
    A yellow tick marks the ground level; bar colour matches stance/swing.
    Returns the Y coordinate below the last bar.
    """
    cv2.putText(img, "── LEG HEIGHTS ──", (x0, y0),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, C_HIGHLIGHT, 1)
    y0 += 24
    row_h = 22

    for i in range(6):
        z      = float(pos_body[i, 2])
        in_air = z > SWING_THRESHOLD
        col    = C_SWING if in_air else C_STANCE

        by = y0 + i * row_h

        # Background slot
        cv2.rectangle(img, (x0, by), (x0 + bar_w, by + 14), C_BAR_BG, -1)

        # Filled bar proportional to Z
        fill_w = int((z - Z_BAR_MIN) / Z_BAR_RANGE * bar_w)
        fill_w = max(0, min(bar_w, fill_w))
        cv2.rectangle(img, (x0, by), (x0 + fill_w, by + 14), col, -1)

        # Ground-level tick
        gnd_x = x0 + int((consts.GROUND_HEIGHT - Z_BAR_MIN) / Z_BAR_RANGE * bar_w)
        cv2.line(img, (gnd_x, by - 2), (gnd_x, by + 16), C_GROUND, 1)

        # Label
        cv2.putText(img, f"L{i}  {z:+.1f} cm", (x0 + bar_w + 6, by + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, C_TEXT, 1)

    return y0 + 6 * row_h + 8


def draw_inferred_groups(img: np.ndarray, pos_body: np.ndarray,
                         x0: int, y0: int) -> int:
    """
    Show inferred gait groups based on which legs are currently airborne.
    Consecutive elevated legs are grouped together and colour-coded.
    Returns the Y coordinate below the section.
    """
    cv2.putText(img, "── INFERRED GROUPS ──", (x0, y0),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, C_HIGHLIGHT, 1)
    y0 += 22

    airborne = [i for i in range(6) if pos_body[i, 2] > SWING_THRESHOLD]
    grounded = [i for i in range(6) if i not in airborne]

    def fmt(legs):
        return "[" + ", ".join(str(l) for l in legs) + "]" if legs else "[  ]"

    # Swing group
    cv2.circle(img, (x0 + 8, y0 - 5), 5, C_SWING, -1)
    cv2.putText(img, f"swing   {fmt(airborne)}", (x0 + 18, y0),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_SWING, 1)
    y0 += 20

    # Stance group
    cv2.circle(img, (x0 + 8, y0 - 5), 5, C_STANCE, -1)
    cv2.putText(img, f"stance  {fmt(grounded)}", (x0 + 18, y0),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_STANCE, 1)
    y0 += 28

    return y0


def draw_panel(img: np.ndarray, state: SimState) -> None:
    """Render the right-hand status / data panel."""
    with state.lock:
        cmd_count = state.cmd_count
        rate_hz   = state.rate_hz
        last_t    = state.last_t
        pos_body  = state.pos_body.copy()

    x0 = DIVIDER_X + 12
    y  = 20
    dy = 22

    def txt(s, col=C_TEXT, scale=0.50, bold=False):
        nonlocal y
        cv2.putText(img, s, (x0, y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, col, 2 if bold else 1)
        y += dy

    # Server status
    txt("── SERVER ──", C_HIGHLIGHT, bold=True)
    txt(f"  Commands : {cmd_count}")
    txt(f"  Rate     : {rate_hz:.1f} Hz")

    age     = time.time() - last_t
    age_col = C_WARN if age > 1.0 else C_TEXT
    txt(f"  Last msg : {age:.2f} s ago", age_col)

    y += 8
    y = draw_height_bars(img, pos_body, x0=x0, y0=y)

    y += 8
    y = draw_inferred_groups(img, pos_body, x0=x0, y0=y)

    # Legend
    y = WIN_H - 54
    txt("── LEGEND ──", C_HIGHLIGHT, bold=True)
    cv2.circle(img, (x0 + 7, y - 5), 6, C_SWING, -1)
    cv2.putText(img, "Swing (airborne)", (x0 + 18, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, C_TEXT, 1)
    y += 18
    cv2.circle(img, (x0 + 7, y - 5), 6, C_STANCE, -1)
    cv2.putText(img, "Stance (ground)",  (x0 + 18, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, C_TEXT, 1)


def build_frame(state: SimState) -> np.ndarray:
    """Composite the full visualisation frame."""
    img = np.full((WIN_H, WIN_W, 3), C_BG, dtype=np.uint8)

    with state.lock:
        pos_body = state.pos_body.copy()

    # Left panel — top-down view
    draw_grid(img)
    draw_body_outline(img)
    draw_legs(img, pos_body)

    # Title
    cv2.putText(img, "Hexapod Sim Server", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, C_HIGHLIGHT, 2)
    cv2.putText(img, "top-down  body frame  5 cm grid", (10, 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, C_DIM, 1)

    # Divider
    cv2.line(img, (DIVIDER_X, 0), (DIVIDER_X, WIN_H), (60, 60, 60), 1)

    # Right panel
    draw_panel(img, state)

    return img


# ── Main ───────────────────────────────────────────────────────────────────────

def _local_ip() -> str:
    """Best-effort guess of the machine's LAN IP address."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return '127.0.0.1'


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Hexapod simulation server",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--host', default='0.0.0.0',
                        help="Bind host (default: 0.0.0.0 = all interfaces)")
    parser.add_argument('--port', type=int, default=8080,
                        help="Listen port (default: 8080)")
    args = parser.parse_args()

    state   = SimState()
    server  = HTTPServer((args.host, args.port), make_handler(state))
    lan_ip  = _local_ip()

    print(f"Sim server listening on  {args.host}:{args.port}")
    print(f"Local network IP         {lan_ip}")
    print()
    print("Point any demo script at this server:")
    print(f"  python src/demo_gait.py    --IP {lan_ip}:{args.port}")
    print(f"  python src/ps_gait_demo.py --IP {lan_ip}:{args.port}")
    print()
    print("Press Q in the window to quit.\n")

    # Serve HTTP in a background daemon thread so the main thread can own OpenCV.
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()

    cv2.namedWindow("Hexapod Sim Server", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Hexapod Sim Server", WIN_W, WIN_H)

    while True:
        frame = build_frame(state)
        cv2.imshow("Hexapod Sim Server", frame)
        key = cv2.waitKey(33) & 0xFF   # ~30 fps
        if key in (ord('q'), ord('Q'), 27):
            break

    server.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
