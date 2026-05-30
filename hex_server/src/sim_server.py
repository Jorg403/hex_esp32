"""
Hexapod Simulation Server — World-Frame Walking View
=====================================================
Behaves like the ESP32 firmware over HTTP.  Start this first, then point
any demo script at --IP 127.0.0.1:<port> to test without hardware.

World-frame rendering:
  • Stance feet are visually anchored to the world — they do not move while
    the leg is on the ground.
  • The body hexagon translates AND rotates as the robot walks/turns.
  • Pose (x, y, heading) is estimated each frame via SVD Procrustes on the
    set of stance feet: q_i = R * p_i + t, where q_i are the fixed world
    positions of stance feet and p_i are their current body-frame positions.
  • A fading trail marks where the body has been.
  • A bounding box shows the total travel extent.
  • Camera is centred on the stance-foot centroid (soft follow).

Usage:
    python src/sim_server.py                    # port 8080, all interfaces
    python src/sim_server.py --port 9090

Then in a separate terminal:
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
    """Return (x, y, z) in leg frame given three joint angles in degrees."""
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


# ── 2-D pose helpers ───────────────────────────────────────────────────────────

def rot2d(theta: float) -> np.ndarray:
    """2×2 rotation matrix for angle theta (radians)."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def estimate_pose_2d(q: np.ndarray, p: np.ndarray):
    """
    Estimate 2-D rigid body pose (translation t, rotation R) such that
        q_i ≈ R @ p_i + t
    using the SVD / Procrustes method.

    Parameters
    ----------
    q : (N, 2)  world-frame positions (known, fixed for stance feet)
    p : (N, 2)  body-frame positions  (from FK)

    Returns
    -------
    t     : (2,) float64   translation (body origin in world frame)
    theta : float          heading angle in radians
    R     : (2,2) float64  rotation matrix

    If N < 2, returns None for all — caller must fall back to previous pose.
    """
    if len(q) < 2:
        return None, None, None

    q = np.asarray(q, dtype=np.float64)
    p = np.asarray(p, dtype=np.float64)

    q_bar = q.mean(axis=0)
    p_bar = p.mean(axis=0)

    q_c = q - q_bar
    p_c = p - p_bar

    H = p_c.T @ q_c          # (2, 2) cross-covariance
    U, _, Vt = np.linalg.svd(H)

    # Enforce proper rotation (det = +1, no reflection)
    d = np.linalg.det(Vt.T @ U.T)
    R = Vt.T @ np.diag([1.0, d]) @ U.T

    t     = q_bar - R @ p_bar
    theta = math.atan2(R[1, 0], R[0, 0])

    return t.astype(np.float32), theta, R


# ── Layout & visual constants ──────────────────────────────────────────────────

WIN_W, WIN_H  = 1020, 660
VIEW_W        = 620          # left-panel pixel width
VIEW_CX       = VIEW_W // 2
VIEW_CY       = WIN_H  // 2
WORLD_SCALE   = 7.0          # px per cm  (smaller → more world visible)
DIVIDER_X     = VIEW_W

TRAIL_LENGTH  = 500          # frames of body history kept
CAM_ALPHA     = 0.05         # camera smoothing (0 = frozen, 1 = instant snap)

SWING_THRESHOLD = consts.GROUND_HEIGHT + 0.5

Z_BAR_MIN   = consts.GROUND_HEIGHT - 1.0
Z_BAR_MAX   = consts.GROUND_HEIGHT + 5.0
Z_BAR_RANGE = Z_BAR_MAX - Z_BAR_MIN

# Colours (BGR)
C_BG        = (30,  30,  30)
C_GRID      = (50,  50,  50)
C_AXES      = (75,  75,  75)
C_BODY      = (140, 140, 140)
C_STANCE    = (60,  200,  60)
C_SWING     = (60,  220, 220)
C_TEXT      = (220, 220, 220)
C_DIM       = (110, 110, 110)
C_HIGHLIGHT = (0,   200, 255)
C_WARN      = (40,  100, 240)
C_BAR_BG    = (55,  55,  55)
C_GROUND    = (160, 160,  60)
C_BBOX      = (90,  90,  200)

# Body→leg transform matrices (ordered 0–5)
_TM = [
    consts.TM_BODY_LEG0, consts.TM_BODY_LEG1, consts.TM_BODY_LEG2,
    consts.TM_BODY_LEG3, consts.TM_BODY_LEG4, consts.TM_BODY_LEG5,
]
# Body-frame mount-point offsets (x, y) extracted from each TM
_MOUNT_BODY = np.array([[tm[0, 3], tm[1, 3]] for tm in _TM], dtype=np.float64)


# ── Shared simulation state ────────────────────────────────────────────────────

class SimState:
    """Thread-safe world-frame odometry state."""

    def __init__(self):
        self.pos_body  = consts.INITIAL_POSITIONS_BODY.copy()
        self.pos_leg   = consts.INITIAL_POSITIONS.copy()

        # World-frame pose
        self.body_world = np.zeros(2, dtype=np.float32)   # (x, y) in world
        self.body_theta = 0.0                              # heading (radians)

        # World-frame foot positions — initialised at body-frame start positions
        # (body is at world origin with zero heading initially)
        self.foot_world = consts.INITIAL_POSITIONS_BODY[:, :2].copy().astype(np.float32)

        self.is_stance  = np.ones(6, dtype=bool)
        self.body_trail: list = []
        self.cam_pos    = np.zeros(2, dtype=np.float32)

        self.cmd_count = 0
        self.last_t    = time.time()
        self._prev_t   = time.time()
        self.rate_hz   = 0.0
        self.lock      = threading.Lock()

    def update(self, angles_flat: list) -> None:
        # ── FK ────────────────────────────────────────────────────────────────
        pos_leg = np.zeros((6, 3), dtype=np.float32)
        for i in range(6):
            pos_leg[i] = fk_leg(angles_flat[i*3], angles_flat[i*3+1], angles_flat[i*3+2])

        pos_body   = leg_to_base(pos_leg, np.eye(4, dtype=np.float32))
        is_stance  = pos_body[:, 2] <= SWING_THRESHOLD
        foot_body2 = pos_body[:, :2]        # (6,2) foot XY in body frame

        with self.lock:
            foot_world = self.foot_world.copy()
            prev_body  = self.body_world.copy()
            prev_theta = self.body_theta

        # ── Pose estimation from stance feet (SVD Procrustes) ─────────────────
        # q_i = R * p_i + t,  q = world, p = body frame
        stance_idx = [i for i in range(6) if is_stance[i]]

        if len(stance_idx) >= 2:
            q = np.array([foot_world[i]  for i in stance_idx])
            p = np.array([foot_body2[i]  for i in stance_idx])
            t, theta, R = estimate_pose_2d(q, p)
            if t is None:           # degenerate (shouldn't happen with ≥2 pts)
                t, theta = prev_body, prev_theta
                R = rot2d(theta)
        elif len(stance_idx) == 1:
            # Single foot: translation only, keep previous heading
            theta = prev_theta
            R     = rot2d(theta)
            i0    = stance_idx[0]
            t     = (foot_world[i0] - (R @ foot_body2[i0])).astype(np.float32)
        else:
            # All airborne: freeze pose
            t, theta = prev_body, prev_theta
            R = rot2d(theta)

        body_world = np.asarray(t, dtype=np.float32)

        # ── Update world positions of swing feet ──────────────────────────────
        new_foot_world = foot_world.copy()
        for i in range(6):
            if not is_stance[i]:
                new_foot_world[i] = (body_world + R @ foot_body2[i]).astype(np.float32)

        # ── Camera target: centroid of stance feet ────────────────────────────
        if stance_idx:
            cam_target = np.array([new_foot_world[i] for i in stance_idx]).mean(axis=0).astype(np.float32)
        else:
            cam_target = body_world.copy()

        now = time.time()
        with self.lock:
            self.pos_leg    = pos_leg
            self.pos_body   = pos_body
            self.is_stance  = is_stance
            self.foot_world = new_foot_world
            self.body_world = body_world
            self.body_theta = float(theta)
            self.cam_pos   += CAM_ALPHA * (cam_target - self.cam_pos)
            self.body_trail.append(body_world.copy())
            if len(self.body_trail) > TRAIL_LENGTH:
                self.body_trail.pop(0)
            self.cmd_count += 1
            dt = now - self._prev_t
            instant = 1.0 / dt if dt > 1e-6 else 0.0
            self.rate_hz = 0.85 * self.rate_hz + 0.15 * instant
            self._prev_t = now
            self.last_t  = now


# ── HTTP handler ───────────────────────────────────────────────────────────────

def make_handler(state: SimState):
    class HexHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            parsed   = urlparse(self.path)
            params   = parse_qs(parsed.query)
            cmd_list = params.get('cmd', [])
            cmd      = cmd_list[0] if cmd_list else ''

            if cmd.startswith('set_positions'):
                angles_str = cmd[len('set_positions'):].strip()
                try:
                    angles = [float(a) for a in angles_str.split(',')]
                    if len(angles) == 18:
                        state.update(angles)
                except ValueError:
                    pass

            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'')

        def log_message(self, fmt, *args):
            pass

    return HexHandler


# ── World-frame drawing helpers ────────────────────────────────────────────────

def w2px(wx: float, wy: float, cam: np.ndarray):
    """World (cm) → pixel coords in the left panel."""
    px = int(VIEW_CX + (wx - cam[0]) * WORLD_SCALE)
    py = int(VIEW_CY - (wy - cam[1]) * WORLD_SCALE)
    return px, py


def draw_grid(img: np.ndarray, cam: np.ndarray) -> None:
    """World-aligned 5 cm grid that scrolls with the camera."""
    sp = 5.0 * WORLD_SCALE

    phase_x = (VIEW_CX - cam[0] * WORLD_SCALE) % sp
    phase_y = (VIEW_CY + cam[1] * WORLD_SCALE) % sp

    x = phase_x
    while x < VIEW_W:
        cv2.line(img, (int(x), 0), (int(x), WIN_H), C_GRID, 1)
        x += sp
    x = phase_x - sp
    while x >= 0:
        cv2.line(img, (int(x), 0), (int(x), WIN_H), C_GRID, 1)
        x -= sp

    y = phase_y
    while y < WIN_H:
        cv2.line(img, (0, int(y)), (VIEW_W, int(y)), C_GRID, 1)
        y += sp
    y = phase_y - sp
    while y >= 0:
        cv2.line(img, (0, int(y)), (VIEW_W, int(y)), C_GRID, 1)
        y -= sp

    ax, _ = w2px(0.0, 0.0, cam)
    _, ay = w2px(0.0, 0.0, cam)
    if 0 < ax < VIEW_W:
        cv2.line(img, (ax, 0), (ax, WIN_H), C_AXES, 1)
    if 0 < ay < WIN_H:
        cv2.line(img, (0, ay), (VIEW_W, ay), C_AXES, 1)


def draw_trail(img: np.ndarray, trail: list, cam: np.ndarray) -> None:
    """Fading amber trail of the body-centre path."""
    n = len(trail)
    if n < 2:
        return
    for i in range(1, n):
        alpha = i / n
        v     = int(40 + 180 * alpha)
        col   = (v, v // 2, v // 4)
        p1 = w2px(trail[i-1][0], trail[i-1][1], cam)
        p2 = w2px(trail[i  ][0], trail[i  ][1], cam)
        cv2.line(img, p1, p2, col, 1)


def draw_bounding_box(img: np.ndarray, trail: list, cam: np.ndarray) -> None:
    """Rectangle enclosing the total body travel extent."""
    if len(trail) < 4:
        return
    xs  = [p[0] for p in trail]
    ys  = [p[1] for p in trail]
    pad = 4.0
    tl  = w2px(min(xs) - pad, max(ys) + pad, cam)
    br  = w2px(max(xs) + pad, min(ys) - pad, cam)
    cv2.rectangle(img, tl, br, C_BBOX, 1)
    dx, dy = max(xs) - min(xs), max(ys) - min(ys)
    cv2.putText(img, f"{dx:.1f} x {dy:.1f} cm",
                (tl[0] + 3, tl[1] - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.34, C_BBOX, 1)


def _mount_world(body_world: np.ndarray, R: np.ndarray):
    """(6,2) world-frame leg mount points given body pose."""
    bxy = body_world.reshape(1, 2).astype(np.float64)
    return bxy + (_MOUNT_BODY @ R.T)   # (6,2)


def draw_body(img: np.ndarray, body_world: np.ndarray,
              theta: float, cam: np.ndarray) -> None:
    """Body hexagon in world frame, rotated by current heading."""
    R    = rot2d(theta)
    mpts = _mount_world(body_world, R)
    pts  = np.array([w2px(mpts[i, 0], mpts[i, 1], cam) for i in range(6)], dtype=np.int32)
    cv2.polylines(img, [pts], isClosed=True, color=C_BODY, thickness=2)

    centre = w2px(float(body_world[0]), float(body_world[1]), cam)
    fwd_w  = body_world.astype(np.float64) + R @ np.array([4.0, 0.0])
    fwd    = w2px(fwd_w[0], fwd_w[1], cam)
    cv2.circle(img, centre, 4, C_BODY, -1)
    cv2.arrowedLine(img, centre, fwd, C_HIGHLIGHT, 2, tipLength=0.4)


def draw_legs(img: np.ndarray, foot_world: np.ndarray,
              body_world: np.ndarray, theta: float,
              is_stance: np.ndarray, cam: np.ndarray) -> None:
    """Leg lines and foot dots in world frame."""
    R    = rot2d(theta)
    mpts = _mount_world(body_world, R)

    for i in range(6):
        mount = w2px(mpts[i, 0], mpts[i, 1], cam)
        foot  = w2px(float(foot_world[i, 0]), float(foot_world[i, 1]), cam)
        col   = C_STANCE if is_stance[i] else C_SWING
        cv2.line(img, mount, foot, col, 1)
        r = 7 if is_stance[i] else 9
        cv2.circle(img, foot, r, col, -1)
        cv2.putText(img, str(i), (foot[0] + 9, foot[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.37, C_TEXT, 1)


# ── Right-panel helpers ────────────────────────────────────────────────────────

def draw_height_bars(img: np.ndarray, pos_body: np.ndarray,
                     x0: int, y0: int, bar_w: int = 200) -> int:
    cv2.putText(img, "── LEG HEIGHTS ──", (x0, y0),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, C_HIGHLIGHT, 1)
    y0 += 24
    row_h = 22
    for i in range(6):
        z      = float(pos_body[i, 2])
        in_air = z > SWING_THRESHOLD
        col    = C_SWING if in_air else C_STANCE
        by_    = y0 + i * row_h
        cv2.rectangle(img, (x0, by_), (x0 + bar_w, by_ + 14), C_BAR_BG, -1)
        fill_w = int((z - Z_BAR_MIN) / Z_BAR_RANGE * bar_w)
        fill_w = max(0, min(bar_w, fill_w))
        cv2.rectangle(img, (x0, by_), (x0 + fill_w, by_ + 14), col, -1)
        gnd_x  = x0 + int((consts.GROUND_HEIGHT - Z_BAR_MIN) / Z_BAR_RANGE * bar_w)
        cv2.line(img, (gnd_x, by_ - 2), (gnd_x, by_ + 16), C_GROUND, 1)
        cv2.putText(img, f"L{i}  {z:+.1f} cm", (x0 + bar_w + 6, by_ + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, C_TEXT, 1)
    return y0 + 6 * row_h + 8


def draw_inferred_groups(img: np.ndarray, pos_body: np.ndarray,
                         x0: int, y0: int) -> int:
    cv2.putText(img, "── INFERRED GROUPS ──", (x0, y0),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, C_HIGHLIGHT, 1)
    y0   += 22
    air    = [i for i in range(6) if pos_body[i, 2] > SWING_THRESHOLD]
    ground = [i for i in range(6) if i not in air]

    def fmt(legs):
        return "[" + ", ".join(str(l) for l in legs) + "]" if legs else "[  ]"

    cv2.circle(img, (x0 + 8, y0 - 5), 5, C_SWING, -1)
    cv2.putText(img, f"swing   {fmt(air)}", (x0 + 18, y0),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_SWING, 1)
    y0 += 20
    cv2.circle(img, (x0 + 8, y0 - 5), 5, C_STANCE, -1)
    cv2.putText(img, f"stance  {fmt(ground)}", (x0 + 18, y0),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_STANCE, 1)
    return y0 + 28


def draw_panel(img: np.ndarray, state: SimState) -> None:
    with state.lock:
        cmd_count  = state.cmd_count
        rate_hz    = state.rate_hz
        last_t     = state.last_t
        pos_body   = state.pos_body.copy()
        body_world = state.body_world.copy()
        body_theta = state.body_theta

    x0 = DIVIDER_X + 12
    y  = 20
    dy = 22

    def txt(s, col=C_TEXT, scale=0.50, bold=False):
        nonlocal y
        cv2.putText(img, s, (x0, y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, col, 2 if bold else 1)
        y += dy

    txt("── SERVER ──", C_HIGHLIGHT, bold=True)
    txt(f"  Commands : {cmd_count}")
    txt(f"  Rate     : {rate_hz:.1f} Hz")
    age     = time.time() - last_t
    age_col = C_WARN if age > 1.0 else C_TEXT
    txt(f"  Last msg : {age:.2f} s ago", age_col)

    y += 4
    txt("── WORLD POSE ──", C_HIGHLIGHT, bold=True)
    txt(f"  X   : {body_world[0]:+7.2f} cm")
    txt(f"  Y   : {body_world[1]:+7.2f} cm")
    txt(f"  Hdg : {math.degrees(body_theta):+7.1f} deg")

    y += 8
    y = draw_height_bars(img, pos_body, x0=x0, y0=y)

    y += 8
    y = draw_inferred_groups(img, pos_body, x0=x0, y0=y)

    y = WIN_H - 54
    txt("── LEGEND ──", C_HIGHLIGHT, bold=True)
    cv2.circle(img, (x0 + 7, y - 5), 6, C_SWING, -1)
    cv2.putText(img, "Swing (airborne)", (x0 + 18, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, C_TEXT, 1)
    y += 18
    cv2.circle(img, (x0 + 7, y - 5), 6, C_STANCE, -1)
    cv2.putText(img, "Stance (ground)", (x0 + 18, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, C_TEXT, 1)


# ── Frame compositor ───────────────────────────────────────────────────────────

def build_frame(state: SimState) -> np.ndarray:
    img = np.full((WIN_H, WIN_W, 3), C_BG, dtype=np.uint8)

    with state.lock:
        pos_body   = state.pos_body.copy()
        foot_world = state.foot_world.copy()
        body_world = state.body_world.copy()
        body_theta = state.body_theta
        is_stance  = state.is_stance.copy()
        trail      = list(state.body_trail)
        cam        = state.cam_pos.copy()

    draw_grid(img, cam)
    draw_trail(img, trail, cam)
    # draw_bounding_box(img, trail, cam)
    draw_legs(img, foot_world, body_world, body_theta, is_stance, cam)
    draw_body(img, body_world, body_theta, cam)

    cv2.putText(img, "Hexapod Sim  — World Frame", (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_HIGHLIGHT, 2)
    bpos = (f"body ({body_world[0]:+.1f}, {body_world[1]:+.1f}) cm  "
            f"hdg {math.degrees(body_theta):+.1f}°   5 cm grid")
    cv2.putText(img, bpos, (10, 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.37, C_DIM, 1)

    # Right panel — blank any left-panel overflow, then draw
    cv2.rectangle(img, (DIVIDER_X, 0), (WIN_W, WIN_H), C_BG, -1)
    cv2.line(img, (DIVIDER_X, 0), (DIVIDER_X, WIN_H), (60, 60, 60), 1)
    draw_panel(img, state)

    return img


# ── Main ───────────────────────────────────────────────────────────────────────

def _local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return '127.0.0.1'


def main() -> None:
    parser = argparse.ArgumentParser(description="Hexapod simulation server")
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()

    state  = SimState()
    server = HTTPServer((args.host, args.port), make_handler(state))
    lan_ip = _local_ip()

    print(f"Sim server listening on  {args.host}:{args.port}")
    print(f"Local network IP         {lan_ip}")
    print()
    print("Point any demo script at this server:")
    print(f"  python src/ps_gait_demo.py --IP {lan_ip}:{args.port}")
    print()
    print("Press Q in the window to quit.\n")

    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()

    cv2.namedWindow("Hexapod Sim Server", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Hexapod Sim Server", WIN_W, WIN_H)

    while True:
        frame = build_frame(state)
        cv2.imshow("Hexapod Sim Server", frame)
        key = cv2.waitKey(33) & 0xFF
        if key in (ord('q'), ord('Q'), 27):
            break

    server.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
