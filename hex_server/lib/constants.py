

SERVO_MIN_POS = [-90, -150, -130]
SERVO_MAX_POS = [90, 30, 50]

DT = 0.05  # Time step in seconds

# GROUND_HEIGHT = -11
GROUND_HEIGHT = -16
MAX_SPEEDS = {"ground": 10.0, "air": 30.0 } # Speeds for ground/air movement in cm/s
BASE_BOUNDING_BOX = [6.5, 8.5, -3.5]  # Bounding box for the robot base in cm

# LEG_LENGTHS = [5.0, 6.0, 10.0]  # Lengths of the legs in cm
LEG_LENGTHS = [5.0, 6.0, 5.5]