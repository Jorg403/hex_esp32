"""
Hexapod Simulation Server — 3-D OpenGL Visualiser
==================================================
Drop-in replacement for sim_server.py.  The HTTP API is identical, so any
demo script that works with the original works here unchanged.

3-D view:
  • Full 3-D world-frame rendering via PyOpenGL + GLFW (real GPU pipeline).
  • Orbit camera: left-drag to rotate, scroll to zoom, right-drag to pan.
  • Stance feet are anchored to the world; body hexagon translates + rotates.
  • Fading amber trail marks the body-centre path through 3-D space.
  • Right-side HUD panel (rendered with OpenGL quads + bitmap text) shows
    the same telemetry as the 2-D version.
  • 5 cm world grid on the XY plane.
  • Leg segments drawn as anti-aliased lines in 3-D (coxa→femur→tibia→foot).

Dependencies:
    pip install PyOpenGL PyOpenGL_accelerate glfw numpy

Usage:
    python sim_server_3d.py                 # port 8080
    python sim_server_3d.py --port 9090

Then in a separate terminal:
    python src/ps_gait_demo.py --IP 127.0.0.1:8080

Press Q or Escape to quit.
"""

import sys, os, argparse, threading, time, math, socket
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

import numpy as np

import glfw
from OpenGL.GL import *
from OpenGL.GLU import *

try:
    _here = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(os.path.abspath(os.path.join(_here, 'src', '..')))
    sys.path.append(_here)
except NameError:
    pass  # running via exec() in tests

# ── Try to import project constants; provide sensible stubs if missing ─────────
try:
    import lib.constants as consts
    from lib.utils.coords_utils import leg_to_base
    _HAS_CONSTS = True
except ImportError:
    _HAS_CONSTS = False

if not _HAS_CONSTS:
    # ── Exact replica of lib/constants.py so the visualiser runs standalone ───
    _GROUND_HEIGHT = -8.0
    _LEG_LENGTHS   = [6.0, 6.0, 10.0]

    _SY = 9.0;  _SR = math.radians(90)
    _CY = 4.85; _CX = 8.7; _CR = math.radians(30)

    def _tm(c, s, tx, ty):
        m = np.eye(4, dtype=np.float32)
        m[0,0]=c; m[0,1]=-s; m[0,3]=tx
        m[1,0]=s; m[1,1]= c; m[1,3]=ty
        return m

    _stub_TM0 = _tm( math.cos(-_CR),        math.sin(-_CR),         _CX, -_CY)
    _stub_TM1 = _tm( math.cos(-_SR),        math.sin(-_SR),         0.0, -_SY)
    _stub_TM2 = _tm( math.cos(_CR-math.pi), math.sin(_CR-math.pi), -_CX, -_CY)
    _stub_TM3 = _tm( math.cos(_CR),         math.sin(_CR),          _CX,  _CY)
    _stub_TM4 = _tm( math.cos(_SR),         math.sin(_SR),          0.0,  _SY)
    _stub_TM5 = _tm( math.cos(math.pi-_CR), math.sin(math.pi-_CR), -_CX,  _CY)
    _stub_tms = [_stub_TM0, _stub_TM1, _stub_TM2,
                 _stub_TM3, _stub_TM4, _stub_TM5]

    _stub_init_leg = np.tile(
        np.array([11.0, 0.0, _GROUND_HEIGHT], dtype=np.float32), (6, 1))
    _stub_init_body = np.array([
        (_stub_tms[i] @ np.append(_stub_init_leg[i], 1.0))[:3]
        for i in range(6)
    ], dtype=np.float32)

    class consts:
        LEG_LENGTHS            = _LEG_LENGTHS
        GROUND_HEIGHT          = _GROUND_HEIGHT
        INITIAL_POSITIONS      = _stub_init_leg
        INITIAL_POSITIONS_BODY = _stub_init_body
        TM_BODY_LEG0           = _stub_TM0
        TM_BODY_LEG1           = _stub_TM1
        TM_BODY_LEG2           = _stub_TM2
        TM_BODY_LEG3           = _stub_TM3
        TM_BODY_LEG4           = _stub_TM4
        TM_BODY_LEG5           = _stub_TM5

    def leg_to_base(pos_leg, _tm_unused):
        """Transform leg-frame foot positions to body frame via each leg TM."""
        out = np.zeros_like(pos_leg)
        for i in range(6):
            out[i] = (_stub_tms[i] @ np.append(pos_leg[i], 1.0))[:3]
        return out



# ── Forward Kinematics ─────────────────────────────────────────────────────────

def fk_leg(t1_deg, t2_deg, t3_deg):
    """Return (x, y, z) in leg frame."""
    L1, L2, L3 = consts.LEG_LENGTHS
    t1 = math.radians(t1_deg)
    t2 = math.radians(t2_deg)
    t3 = math.radians(t3_deg)
    phi2 = t2 + math.pi / 2
    phi3 = t2 - t3
    s  = L2 * math.cos(phi2) + L3 * math.cos(phi3)
    z  = L2 * math.sin(phi2) + L3 * math.sin(phi3)
    r  = s + L1
    return np.array([ r*math.cos(t1), -r*math.sin(t1), z], dtype=np.float32)


def fk_leg_segments(t1_deg, t2_deg, t3_deg):
    """
    Return the 4 key points of a leg in leg-frame coordinates:
        [origin, after_coxa, after_femur, tip]
    Used for drawing coxa / femur / tibia segments separately.
    """
    L1, L2, L3 = consts.LEG_LENGTHS
    t1 = math.radians(t1_deg)
    t2 = math.radians(t2_deg)
    t3 = math.radians(t3_deg)

    # Coxa end (just L1 along the hip direction in XY)
    cx = L1 * math.cos(t1);  cy = -L1 * math.sin(t1);  cz = 0.0

    # Femur end
    phi2 = t2 + math.pi / 2
    fx = cx + L2 * math.cos(phi2) * math.cos(t1)
    fy = cy - L2 * math.cos(phi2) * math.sin(t1)
    fz = cz + L2 * math.sin(phi2)

    # Tibia end (= tip)
    phi3 = t2 - t3
    tx = fx + L3 * math.cos(phi3) * math.cos(t1)
    ty = fy - L3 * math.cos(phi3) * math.sin(t1)
    tz = fz + L3 * math.sin(phi3)

    return np.array([
        [0.0, 0.0, 0.0],
        [cx,  cy,  cz ],
        [fx,  fy,  fz ],
        [tx,  ty,  tz ],
    ], dtype=np.float32)


# ── 2-D pose helpers (unchanged from original) ─────────────────────────────────

def rot2d(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c,-s],[s,c]], dtype=np.float64)

def estimate_pose_2d(q, p):
    if len(q) < 2:
        return None, None, None
    q = np.asarray(q, dtype=np.float64)
    p = np.asarray(p, dtype=np.float64)
    q_bar, p_bar = q.mean(0), p.mean(0)
    H = (p - p_bar).T @ (q - q_bar)
    U, _, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    t = q_bar - R @ p_bar
    return t.astype(np.float32), math.atan2(R[1,0], R[0,0]), R


# ── Simulation constants ────────────────────────────────────────────────────────

SWING_THRESHOLD = consts.GROUND_HEIGHT + 0.5
TRAIL_LENGTH    = 600

_TM = [
    consts.TM_BODY_LEG0, consts.TM_BODY_LEG1, consts.TM_BODY_LEG2,
    consts.TM_BODY_LEG3, consts.TM_BODY_LEG4, consts.TM_BODY_LEG5,
]
_MOUNT_BODY = np.array([[tm[0,3], tm[1,3]] for tm in _TM], dtype=np.float64)
# Also keep z-offset of mounts (usually 0)
_MOUNT_BODY_Z = np.array([tm[2,3] for tm in _TM], dtype=np.float64)


# ── Shared state ───────────────────────────────────────────────────────────────

class SimState:
    def __init__(self):
        self.pos_body   = consts.INITIAL_POSITIONS_BODY.copy()
        self.pos_leg    = consts.INITIAL_POSITIONS.copy()
        # Leg segment points in body frame: shape (6, 4, 3)
        self.seg_body   = np.zeros((6, 4, 3), dtype=np.float32)

        self.body_world = np.zeros(2, dtype=np.float32)
        self.body_theta = 0.0
        self.foot_world = consts.INITIAL_POSITIONS_BODY[:,:2].copy().astype(np.float32)
        self.is_stance  = np.ones(6, dtype=bool)
        self.body_trail = []          # list of (x,y,z) world positions
        self.cam_pos    = np.zeros(2, dtype=np.float32)

        self.cmd_count  = 0
        self.rate_hz    = 0.0
        self.last_t     = time.time()
        self._prev_t    = time.time()
        self.lock       = threading.Lock()

        # Raw joint angles (for segment FK)
        self.angles     = [0.0] * 18

    def update(self, angles_flat):
        pos_leg  = np.zeros((6,3), dtype=np.float32)
        seg_body = np.zeros((6,4,3), dtype=np.float32)
        for i in range(6):
            a1, a2, a3 = angles_flat[i*3], angles_flat[i*3+1], angles_flat[i*3+2]
            pos_leg[i] = fk_leg(a1, a2, a3)
            # Segment points in leg frame → body frame via TM
            segs_lf = fk_leg_segments(a1, a2, a3)   # (4, 3) in leg frame
            TM = _TM[i]
            R3 = TM[:3,:3].astype(np.float32)
            t3 = TM[:3, 3].astype(np.float32)
            seg_body[i] = (segs_lf @ R3.T) + t3     # (4, 3) in body frame

        pos_body  = leg_to_base(pos_leg, np.eye(4, dtype=np.float32))
        is_stance = pos_body[:,2] <= SWING_THRESHOLD
        foot_body2 = pos_body[:,:2]

        with self.lock:
            foot_world = self.foot_world.copy()
            prev_body  = self.body_world.copy()
            prev_theta = self.body_theta

        stance_idx = [i for i in range(6) if is_stance[i]]
        if len(stance_idx) >= 2:
            q = np.array([foot_world[i] for i in stance_idx])
            p = np.array([foot_body2[i] for i in stance_idx])
            t, theta, R = estimate_pose_2d(q, p)
            if t is None: t, theta, R = prev_body, prev_theta, rot2d(prev_theta)
        elif len(stance_idx) == 1:
            theta = prev_theta; R = rot2d(theta)
            i0 = stance_idx[0]
            t  = (foot_world[i0] - R @ foot_body2[i0]).astype(np.float32)
        else:
            t, theta, R = prev_body, prev_theta, rot2d(prev_theta)

        body_world = np.asarray(t, dtype=np.float32)

        new_foot_world = foot_world.copy()
        for i in range(6):
            if not is_stance[i]:
                new_foot_world[i] = (body_world + R @ foot_body2[i]).astype(np.float32)

        # Body height (average of stance feet z in body frame)
        if stance_idx:
            body_z = float(np.mean([pos_body[i,2] for i in stance_idx]))
        else:
            body_z = float(pos_body[:,2].mean())

        now = time.time()
        with self.lock:
            self.pos_leg    = pos_leg
            self.pos_body   = pos_body
            self.seg_body   = seg_body
            self.is_stance  = is_stance
            self.foot_world = new_foot_world
            self.body_world = body_world
            self.body_theta = float(theta)
            self.body_z     = body_z
            self.body_trail.append(np.array([body_world[0], body_world[1], body_z+1.0],
                                             dtype=np.float32))
            if len(self.body_trail) > TRAIL_LENGTH:
                self.body_trail.pop(0)
            self.cmd_count += 1
            dt = now - self._prev_t
            self.rate_hz = 0.85*self.rate_hz + 0.15*(1.0/dt if dt>1e-6 else 0)
            self._prev_t = now
            self.last_t  = now
            self.angles  = angles_flat[:]


# ── HTTP handler ───────────────────────────────────────────────────────────────

def make_handler(state):
    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            params = parse_qs(urlparse(self.path).query)
            cmd    = (params.get('cmd',[''])[0])
            if cmd.startswith('set_positions'):
                try:
                    angles = [float(a) for a in cmd[len('set_positions'):].strip().split(',')]
                    if len(angles) == 18:
                        state.update(angles)
                except ValueError:
                    pass
            self.send_response(200)
            self.send_header('Content-Type','text/plain')
            self.end_headers()
            self.wfile.write(b'')
        def log_message(self,*a): pass
    return H


# ── Camera ─────────────────────────────────────────────────────────────────────

class OrbitCamera:
    """Orbit around a target point.  Left-drag=rotate, scroll=zoom, right-drag=pan."""

    def __init__(self):
        self.azimuth   =  45.0   # degrees
        self.elevation =  30.0   # degrees
        self.distance  = 80.0    # cm
        self.target    = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        self._drag_btn  = None
        self._last_xy   = (0, 0)

    # ── GLFW callbacks ─────────────────────────────────────────────────────────

    def mouse_button_cb(self, window, button, action, mods):
        if action == glfw.PRESS:
            self._drag_btn = button
            self._last_xy  = glfw.get_cursor_pos(window)
        elif action == glfw.RELEASE:
            self._drag_btn = None

    def cursor_pos_cb(self, window, x, y):
        dx = x - self._last_xy[0]
        dy = y - self._last_xy[1]
        self._last_xy = (x, y)
        if self._drag_btn == glfw.MOUSE_BUTTON_LEFT:
            self.azimuth   -= dx * 0.4
            self.elevation  = max(-89, min(89, self.elevation - dy * 0.4))
        elif self._drag_btn == glfw.MOUSE_BUTTON_RIGHT:
            # Pan in the camera's local XY
            a  = math.radians(self.azimuth)
            el = math.radians(self.elevation)
            right   = np.array([ math.cos(a), math.sin(a), 0.0])
            up      = np.array([-math.sin(a)*math.sin(el),
                                  math.cos(a)*math.sin(el),
                                  math.cos(el)])
            speed   = self.distance * 0.002
            self.target -= right * dx * speed
            self.target += up    * dy * speed

    def scroll_cb(self, window, xoff, yoff):
        self.distance = max(10.0, self.distance * (0.9 ** yoff))

    def soft_follow(self, target_xy, alpha=0.04):
        """Smoothly move target to follow the robot."""
        self.target[0] += alpha * (target_xy[0] - self.target[0])
        self.target[1] += alpha * (target_xy[1] - self.target[1])

    # ── Apply ──────────────────────────────────────────────────────────────────

    def apply(self):
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        a  = math.radians(self.azimuth)
        el = math.radians(self.elevation)
        ex = self.target[0] + self.distance * math.cos(el) * math.cos(a)
        ey = self.target[1] + self.distance * math.cos(el) * math.sin(a)
        ez = self.target[2] + self.distance * math.sin(el)
        gluLookAt(ex, ey, ez,
                  self.target[0], self.target[1], self.target[2],
                  0, 0, 1)


# ── OpenGL drawing helpers ─────────────────────────────────────────────────────

def set_proj(w, h, fov=45.0, near=1.0, far=5000.0):
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(fov, w/h, near, far)


def draw_grid(span=100, step=5):
    """XY-plane grid centred at origin."""
    glLineWidth(1.0)
    glBegin(GL_LINES)
    z = consts.GROUND_HEIGHT - 0.5
    for x in np.arange(-span, span+step, step):
        alpha = 0.15 if (x % 20) != 0 else 0.30
        glColor4f(0.9, 0.9, 0.9, alpha)
        glVertex3f(x, -span, z); glVertex3f(x,  span, z)
    for y in np.arange(-span, span+step, step):
        alpha = 0.15 if (y % 20) != 0 else 0.30
        glColor4f(0.9, 0.9, 0.9, alpha)
        glVertex3f(-span, y, z); glVertex3f( span, y, z)
    glEnd()

    # World origin axes
    glLineWidth(2.0)
    glBegin(GL_LINES)
    glColor3f(0.8, 0.2, 0.2); glVertex3f(0,0,z); glVertex3f(10,0,z)   # X red
    glColor3f(0.2, 0.8, 0.2); glVertex3f(0,0,z); glVertex3f(0,10,z)   # Y green
    glColor3f(0.3, 0.5, 1.0); glVertex3f(0,0,z); glVertex3f(0,0,z+10) # Z blue
    glEnd()


def draw_trail(trail):
    n = len(trail)
    if n < 2: return
    glLineWidth(1.5)
    glBegin(GL_LINE_STRIP)
    for i, pt in enumerate(trail):
        alpha  = i / n
        bright = 0.15 + 0.8 * alpha
        glColor4f(bright, bright*0.55, bright*0.15, alpha)
        glVertex3f(*pt)
    glEnd()


def body_hex_world(body_world, body_z, theta):
    """6 world-frame mount points of the body hexagon."""
    R  = rot2d(theta)
    xy = (np.array([[body_world[0], body_world[1]]]) +
          (_MOUNT_BODY @ R.T))          # (6, 2)
    z  = body_z + _MOUNT_BODY_Z        # (6,) — usually all body_z
    return np.column_stack([xy, z])    # (6, 3)


def draw_body(body_world, body_z, theta):
    """Draw the body hexagon outline + heading arrow in 3-D."""
    pts = body_hex_world(body_world, body_z, theta)  # (6,3)

    glLineWidth(2.5)
    glColor3f(0.60, 0.60, 0.60)
    glBegin(GL_LINE_LOOP)
    for p in pts:
        glVertex3f(*p)
    glEnd()

    # Body centre dot
    cx, cy = float(body_world[0]), float(body_world[1])
    cz     = body_z + 1.0
    glPointSize(6.0)
    glColor3f(0.75, 0.75, 0.75)
    glBegin(GL_POINTS)
    glVertex3f(cx, cy, cz)
    glEnd()

    # Forward arrow
    R   = rot2d(theta)
    fwd = R @ np.array([5.0, 0.0])
    glLineWidth(2.5)
    glColor3f(0.0, 0.78, 1.0)
    glBegin(GL_LINES)
    glVertex3f(cx, cy, cz)
    glVertex3f(cx + fwd[0], cy + fwd[1], cz)
    glEnd()


def draw_legs_3d(seg_body, body_world, body_z, theta, foot_world, is_stance):
    """
    Draw each leg as three line segments (coxa / femur / tibia) in 3-D.
    seg_body: (6, 4, 3) — four key points per leg in body frame.
    We transform them into world frame using the current body pose.
    """
    R2   = rot2d(theta)
    # Full 3×3 rotation (body has no pitch/roll in this sim)
    R3   = np.eye(3, dtype=np.float64)
    R3[:2,:2] = R2
    bxyz = np.array([body_world[0], body_world[1], body_z], dtype=np.float64)

    glLineWidth(2.0)
    for i in range(6):
        stance = bool(is_stance[i])
        # Stance: bright green segments; Swing: cyan
        if stance:
            colours = [
                (0.25, 0.85, 0.25),   # coxa
                (0.30, 0.75, 0.30),   # femur
                (0.20, 0.90, 0.20),   # tibia
            ]
        else:
            colours = [
                (0.25, 0.85, 0.85),
                (0.30, 0.75, 0.85),
                (0.20, 0.85, 0.90),
            ]

        # Transform 4 points: body frame → world frame
        pts_w = []
        for k in range(4):
            bf  = seg_body[i,k].astype(np.float64)
            wf  = R3 @ bf + bxyz
            pts_w.append(wf)

        glBegin(GL_LINES)
        for k in range(3):
            glColor3f(*colours[k])
            glVertex3f(*pts_w[k])
            glVertex3f(*pts_w[k+1])
        glEnd()

        # Foot dot
        foot_xy = foot_world[i]
        foot_z  = float(seg_body[i, 3, 2]) + body_z   # z from body-frame seg tip
        fpt     = np.array([float(foot_xy[0]), float(foot_xy[1]), foot_z])
        r       = 3.5 if stance else 4.5
        glPointSize(r * 2)
        if stance:
            glColor3f(0.20, 0.95, 0.20)
        else:
            glColor3f(0.20, 0.90, 0.95)
        glBegin(GL_POINTS)
        glVertex3f(*fpt)
        glEnd()

        # Ground contact marker (flat circle approximated as cross)
        if stance:
            gnd_z = consts.GROUND_HEIGHT - 0.2
            glLineWidth(1.0)
            glColor4f(0.3, 0.9, 0.3, 0.5)
            glBegin(GL_LINES)
            glVertex3f(float(foot_xy[0])-2, float(foot_xy[1]),   gnd_z)
            glVertex3f(float(foot_xy[0])+2, float(foot_xy[1]),   gnd_z)
            glVertex3f(float(foot_xy[0]),   float(foot_xy[1])-2, gnd_z)
            glVertex3f(float(foot_xy[0]),   float(foot_xy[1])+2, gnd_z)
            glEnd()


# ── Minimal bitmap HUD ─────────────────────────────────────────────────────────
# We render the info panel as 2-D overlay using orthographic projection.

def begin_hud(w, h):
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity()
    glOrtho(0, w, 0, h, -1, 1)
    glMatrixMode(GL_MODELVIEW);  glPushMatrix(); glLoadIdentity()
    glDisable(GL_DEPTH_TEST)


def end_hud():
    glEnable(GL_DEPTH_TEST)
    glMatrixMode(GL_PROJECTION); glPopMatrix()
    glMatrixMode(GL_MODELVIEW);  glPopMatrix()


def draw_rect(x, y, ww, hh, r, g, b, a=0.85):
    glColor4f(r, g, b, a)
    glBegin(GL_QUADS)
    glVertex2f(x,    y)
    glVertex2f(x+ww, y)
    glVertex2f(x+ww, y+hh)
    glVertex2f(x,    y+hh)
    glEnd()


# ── Minimal 5×7 bitmap font (ASCII 32–126) — no GLUT required ─────────────────
# Each character is 5 columns × 7 rows; stored as 5 bytes, LSB = top row.
_FONT5X7 = {
    ' ':(0,0,0,0,0),'!':(4,4,4,0,4),'"':(10,10,0,0,0),'#':(10,31,10,31,10),
    '$':(14,21,14,20,14),'%':(17,8,4,2,17),'&':(12,18,9,22,13),
    "'":(4,4,0,0,0),'(':(4,8,8,8,4),')':(4,2,2,2,4),'*':(21,14,31,14,21),
    '+':(0,4,14,4,0),',':(0,0,0,4,8),'-':(0,0,14,0,0),'.':(0,0,0,0,4),
    '/':(1,2,4,8,16),
    '0':(14,17,19,21,14),'1':(4,12,4,4,14),'2':(14,17,2,8,31),
    '3':(31,2,6,17,14),'4':(2,6,10,31,2),'5':(31,16,30,1,30),
    '6':(6,8,30,17,14),'7':(31,1,2,4,8),'8':(14,17,14,17,14),
    '9':(14,17,15,1,14),
    ':':(0,4,0,4,0),';':(0,4,0,4,8),'<':(2,4,8,4,2),
    '=':(0,14,0,14,0),'>':(8,4,2,4,8),'?':(14,17,2,0,4),
    '@':(14,17,13,21,14),
    'A':(14,17,31,17,17),'B':(30,17,30,17,30),'C':(14,17,16,17,14),
    'D':(28,18,17,18,28),'E':(31,16,28,16,31),'F':(31,16,28,16,16),
    'G':(14,17,16,19,14),'H':(17,17,31,17,17),'I':(14,4,4,4,14),
    'J':(7,2,2,18,12),'K':(18,20,24,20,18),'L':(16,16,16,16,31),
    'M':(17,27,21,17,17),'N':(17,25,21,19,17),'O':(14,17,17,17,14),
    'P':(30,17,30,16,16),'Q':(14,17,17,21,14),'R':(30,17,30,18,17),
    'S':(15,16,14,1,30),'T':(31,4,4,4,4),'U':(17,17,17,17,14),
    'V':(17,17,17,10,4),'W':(17,17,21,27,17),'X':(17,10,4,10,17),
    'Y':(17,10,4,4,4),'Z':(31,2,4,8,31),
    '[':(14,8,8,8,14),'\\':(16,8,4,2,1),']':(14,2,2,2,14),
    '^':(4,10,17,0,0),'_':(0,0,0,0,31),'`':(8,4,0,0,0),
    'a':(0,14,1,15,15),'b':(16,16,30,17,30),'c':(0,14,16,16,14),
    'd':(1,1,15,17,15),'e':(0,14,31,16,14),'f':(6,8,28,8,8),
    'g':(0,15,17,15,1),'h':(16,16,30,17,17),'i':(4,0,4,4,14),
    'j':(2,0,2,2,12),'k':(16,18,28,18,17),'l':(12,4,4,4,14),
    'm':(0,27,21,21,17),'n':(0,30,17,17,17),'o':(0,14,17,17,14),
    'p':(0,30,17,30,16),'q':(0,15,17,15,1),'r':(0,14,16,16,16),
    's':(0,15,16,1,30),'t':(8,31,8,8,7),'u':(0,17,17,17,15),
    'v':(0,17,17,10,4),'w':(0,17,21,21,10),'x':(0,17,10,10,17),
    'y':(0,17,15,1,14),'z':(0,31,2,8,31),
    '{':(6,4,8,4,6),'|':(4,4,4,4,4),'}':(12,4,2,4,12),'~':(0,8,21,2,0),
}
_CHAR_W, _CHAR_H, _CHAR_GAP = 5, 7, 2


def gl_text(x, y, text, r=1.0, g=1.0, b=1.0, scale=2):
    """
    Draw ASCII text at (x, y) using a hand-coded 5×7 pixel font.
    Rendered as GL_QUADS — no GLUT, no external fonts needed.
    `scale` = pixel size in screen units (2 gives a readable ~14 px height).
    """
    glColor3f(r, g, b)
    cx = float(x)
    ps = float(scale)
    for ch in text:
        bits = _FONT5X7.get(ch, _FONT5X7.get(' ', (0,0,0,0,0)))
        glBegin(GL_QUADS)
        for col, byte in enumerate(bits):
            for row in range(_CHAR_H):
                if byte & (1 << (_CHAR_H - 1 - row)):
                    px = cx + col * ps
                    py = float(y) - row * ps
                    glVertex2f(px,      py)
                    glVertex2f(px + ps, py)
                    glVertex2f(px + ps, py - ps)
                    glVertex2f(px,      py - ps)
        glEnd()
        cx += (_CHAR_W + _CHAR_GAP) * ps


def draw_hud(w, h, state: SimState):
    """2-D overlay panel on the right side."""
    with state.lock:
        cmd_count  = state.cmd_count
        rate_hz    = state.rate_hz
        last_t     = state.last_t
        body_world = state.body_world.copy()
        body_theta = state.body_theta
        pos_body   = state.pos_body.copy()
        is_stance  = state.is_stance.copy()

    panel_w = 220
    px = w - panel_w - 4
    py = 4

    begin_hud(w, h)

    # Panel background
    draw_rect(px-4, py, panel_w+8, h-8, 0.08, 0.08, 0.10, 0.82)
    # Divider line
    glColor4f(0.25, 0.25, 0.25, 1.0)
    glLineWidth(1.0)
    glBegin(GL_LINES)
    glVertex2f(px-4, py); glVertex2f(px-4, h-4)
    glEnd()

    lh = 18   # line height px  (font is 7*scale=14 px tall, +4 leading)
    tx = px
    ty = h - 20

    def line(txt, r=0.85, g=0.85, b=0.85):
        nonlocal ty
        gl_text(tx, ty, txt, r, g, b)
        ty -= lh

    def section(title):
        nonlocal ty
        ty -= 4
        gl_text(tx, ty, title, 0.0, 0.78, 1.0)
        ty -= lh

    section("── SERVER ──")
    line(f"  Commands : {cmd_count}")
    line(f"  Rate     : {rate_hz:.1f} Hz")
    age = time.time() - last_t
    if age > 1.0:
        line(f"  Last msg : {age:.2f}s ago", 0.9, 0.4, 0.2)
    else:
        line(f"  Last msg : {age:.2f}s ago")

    section("── WORLD POSE ──")
    line(f"  X   : {body_world[0]:+7.2f} cm")
    line(f"  Y   : {body_world[1]:+7.2f} cm")
    line(f"  Hdg : {math.degrees(body_theta):+7.1f} deg")

    section("── LEG HEIGHTS ──")
    for i in range(6):
        z     = float(pos_body[i, 2])
        swing = z > SWING_THRESHOLD
        label = f"  L{i}: {z:+.1f} cm"
        if swing:
            line(label, 0.25, 0.90, 0.90)
        else:
            line(label, 0.25, 0.90, 0.25)

    section("── STANCE ──")
    air    = [str(i) for i in range(6) if not is_stance[i]]
    ground = [str(i) for i in range(6) if     is_stance[i]]
    line(f"  Swing  : [{', '.join(air    or ['-'])}]", 0.25, 0.90, 0.90)
    line(f"  Stance : [{', '.join(ground or ['-'])}]", 0.25, 0.90, 0.25)

    section("── CAMERA ──")
    line("  LMB drag  : rotate")
    line("  RMB drag  : pan")
    line("  Scroll    : zoom")
    line("  Q/Esc     : quit")

    end_hud()


# ── Local IP helper ────────────────────────────────────────────────────────────

def _local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80)); ip = s.getsockname()[0]; s.close()
        return ip
    except OSError:
        return '127.0.0.1'


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()

    # ── HTTP server ────────────────────────────────────────────────────────────
    state  = SimState()
    server = HTTPServer((args.host, args.port), make_handler(state))
    lan_ip = _local_ip()

    print(f"Sim server listening on  {args.host}:{args.port}")
    print(f"Local network IP         {lan_ip}")
    print()
    print("Point any demo script at:")
    print(f"  python src/ps_gait_demo.py --IP {lan_ip}:{args.port}")
    print()
    print("Controls:  LMB drag=rotate  RMB drag=pan  scroll=zoom  Q=quit\n")

    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()

    # ── GLFW / OpenGL init ─────────────────────────────────────────────────────
    if not glfw.init():
        raise RuntimeError("Failed to initialise GLFW")

    glfw.window_hint(glfw.SAMPLES, 4)          # 4× MSAA
    WIN_W, WIN_H = 1280, 720
    window = glfw.create_window(WIN_W, WIN_H, "Hexapod 3-D Sim", None, None)
    if not window:
        glfw.terminate(); raise RuntimeError("Failed to create GLFW window")

    glfw.make_context_current(window)
    glfw.swap_interval(1)                      # vsync

    cam = OrbitCamera()
    glfw.set_mouse_button_callback(window, cam.mouse_button_cb)
    glfw.set_cursor_pos_callback  (window, cam.cursor_pos_cb)
    glfw.set_scroll_callback      (window, cam.scroll_cb)

    glEnable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glEnable(GL_LINE_SMOOTH)
    glEnable(GL_POINT_SMOOTH)
    glHint(GL_LINE_SMOOTH_HINT,  GL_NICEST)
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST)

    # ── Render loop ────────────────────────────────────────────────────────────
    while not glfw.window_should_close(window):
        glfw.poll_events()

        if glfw.get_key(window, glfw.KEY_Q)      == glfw.PRESS: break
        if glfw.get_key(window, glfw.KEY_ESCAPE) == glfw.PRESS: break

        fw, fh = glfw.get_framebuffer_size(window)
        glViewport(0, 0, fw, fh)

        glClearColor(0.11, 0.11, 0.13, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # ── 3-D scene ──────────────────────────────────────────────────────────
        set_proj(fw, fh)

        with state.lock:
            pos_body   = state.pos_body.copy()
            seg_body   = state.seg_body.copy()
            foot_world = state.foot_world.copy()
            body_world = state.body_world.copy()
            body_theta = state.body_theta
            is_stance  = state.is_stance.copy()
            trail      = list(state.body_trail)
            body_z     = getattr(state, 'body_z', consts.GROUND_HEIGHT + 13.0)

        # Soft-follow camera
        cam.soft_follow(body_world, alpha=0.03)
        cam.apply()

        draw_grid()
        draw_trail(trail)
        draw_body(body_world, body_z, body_theta)
        draw_legs_3d(seg_body, body_world, body_z, body_theta,
                     foot_world, is_stance)

        # ── HUD overlay ────────────────────────────────────────────────────────
        draw_hud(fw, fh, state)

        glfw.swap_buffers(window)

    server.shutdown()
    glfw.terminate()


if __name__ == "__main__":
    main()