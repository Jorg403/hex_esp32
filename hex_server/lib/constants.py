import numpy as np

SERVO_MIN_POS = [-90, -150, -130]
SERVO_MAX_POS = [90, 30, 50]

DT = 0.05  # Time step in seconds
# DT = 0.01  # ONLY FOR SIMULATION

# GROUND_HEIGHT = -11
GROUND_HEIGHT = -8
GENERAL_SPEED = 50
# GENERAL_SPEED = 50 # ONLY FOR SIMULATION
IDLING_SPEED = 0.005 * GENERAL_SPEED  # Speed when idling in cm/s
MAX_SPEEDS = {"ground": GENERAL_SPEED*1.0, "air": GENERAL_SPEED*3.0 } # Speeds for ground/air movement in cm/s
BASE_BOUNDING_BOX = [6.5, 8.5, -3.5]  # Bounding box for the robot base in cm

STEP_LENGTH = 5.0  # half-stroke in cm  (foot travels ±stp/2 from home)
LIFT_HEIGHT = 3.0   # foot lift height in cm

# LEG_LENGTHS = [5.0, 6.0, 10.0]  # Lengths of the legs in cm
# LEG_LENGTHS = [6.0, 6.0, 5.9]
LEG_LENGTHS = [6.0, 6.0, 10.0]

ENGINE1_HEIGHT = 0

# Transformation matrices for leg positions relative to the base
__SIDES_OFFSET_Y__ = 9.0  # offset in y for legs 1 and 4
__SIDES_OFFSET_X__ = 0.0  # offset in x for legs 1 and 4
__SIDES_ROTATION__ = np.radians(90)
TM_BODY_LEG1 = np.array([
    [np.cos(-__SIDES_ROTATION__), -np.sin(-__SIDES_ROTATION__), 0, __SIDES_OFFSET_X__],
    [np.sin(-__SIDES_ROTATION__), np.cos(-__SIDES_ROTATION__), 0, -__SIDES_OFFSET_Y__],
    [0, 0, 1, ENGINE1_HEIGHT],
    [0, 0, 0, 1]
], dtype=np.float32)

TM_BODY_LEG4 = np.array([
    [np.cos(__SIDES_ROTATION__), -np.sin(__SIDES_ROTATION__), 0, __SIDES_OFFSET_X__],
    [np.sin(__SIDES_ROTATION__), np.cos(__SIDES_ROTATION__), 0, __SIDES_OFFSET_Y__],
    [0, 0, 1, ENGINE1_HEIGHT],
    [0, 0, 0, 1]
], dtype=np.float32)

__CORNERS_OFFSET_Y__ = 4.85  # offset in y for legs 0 and 3
__CORNERS_OFFSET_X__ = 8.7  # offset in x for legs 0 and 3
__CORNERS_ROTATION__ = np.radians(30)
TM_BODY_LEG0 = np.array([
    [np.cos(-__CORNERS_ROTATION__), -np.sin(-__CORNERS_ROTATION__), 0, __CORNERS_OFFSET_X__],
    [np.sin(-__CORNERS_ROTATION__), np.cos(-__CORNERS_ROTATION__), 0, -__CORNERS_OFFSET_Y__],
    [0, 0, 1, ENGINE1_HEIGHT],
    [0, 0, 0, 1]
], dtype=np.float32)

TM_BODY_LEG2 = np.array([
    [np.cos(__CORNERS_ROTATION__-np.pi), -np.sin(__CORNERS_ROTATION__-np.pi), 0, -__CORNERS_OFFSET_X__],
    [np.sin(__CORNERS_ROTATION__-np.pi), np.cos(__CORNERS_ROTATION__-np.pi), 0, -__CORNERS_OFFSET_Y__],
    [0, 0, 1, ENGINE1_HEIGHT],
    [0, 0, 0, 1]
], dtype=np.float32)

TM_BODY_LEG3 = np.array([
    [np.cos(__CORNERS_ROTATION__), -np.sin(__CORNERS_ROTATION__), 0, __CORNERS_OFFSET_X__],
    [np.sin(__CORNERS_ROTATION__), np.cos(__CORNERS_ROTATION__), 0, __CORNERS_OFFSET_Y__],
    [0, 0, 1, ENGINE1_HEIGHT],
    [0, 0, 0, 1]
], dtype=np.float32)

TM_BODY_LEG5 = np.array([
    [np.cos(np.pi-__CORNERS_ROTATION__), -np.sin(np.pi-__CORNERS_ROTATION__), 0, -__CORNERS_OFFSET_X__],
    [np.sin(np.pi-__CORNERS_ROTATION__), np.cos(np.pi-__CORNERS_ROTATION__), 0, __CORNERS_OFFSET_Y__],
    [0, 0, 1, ENGINE1_HEIGHT],
    [0, 0, 0, 1]
], dtype=np.float32)

# Precalculation of the inverses
TM_LEG0_BODY = np.linalg.inv(TM_BODY_LEG0)
TM_LEG1_BODY = np.linalg.inv(TM_BODY_LEG1)
TM_LEG2_BODY = np.linalg.inv(TM_BODY_LEG2)
TM_LEG3_BODY = np.linalg.inv(TM_BODY_LEG3)
TM_LEG4_BODY = np.linalg.inv(TM_BODY_LEG4)
TM_LEG5_BODY = np.linalg.inv(TM_BODY_LEG5)

INITIAL_POSITIONS = np.tile(np.array([11.0, 0, GROUND_HEIGHT], dtype=np.float32), (6, 1))

INITIAL_POSITIONS_BODY = np.array([
    TM_BODY_LEG0 @ np.append(INITIAL_POSITIONS[0], 1.0),
    TM_BODY_LEG1 @ np.append(INITIAL_POSITIONS[1], 1.0),
    TM_BODY_LEG2 @ np.append(INITIAL_POSITIONS[2], 1.0),
    TM_BODY_LEG3 @ np.append(INITIAL_POSITIONS[3], 1.0),
    TM_BODY_LEG4 @ np.append(INITIAL_POSITIONS[4], 1.0),
    TM_BODY_LEG5 @ np.append(INITIAL_POSITIONS[5], 1.0),
], dtype=np.float32)[:, :3]  # [:, :3] para quitar la coordenada homogénea


# # exaple for use of leg 1
# TM_BASE_TO_LEG1 = TM_BASE_LEG1
# coords_point_leg1 = np.array([1.0, 1.0, 1.0, 1.0], dtype=np.float32)  # Point in leg 1 coordinates
# coords_point_global = TM_BASE_TO_LEG1 @ coords_point_leg1  # Transform to global coordinates
# # the result would be coords_point_global = np.array([1.0, -13.0, 1.0, 1.0], dtype=np.float32)

