# Configuration file for GPIO pin assignments, motor inversions, conotrol constants, and other thresholds.

# Motors
MOTOR_LEFT_DIR_PIN  = 4
MOTOR_LEFT_PWM_PIN  = 5
MOTOR_RIGHT_DIR_PIN = 7
MOTOR_RIGHT_PWM_PIN = 6

# Motor inversion
MOTOR_LEFT_INVERT  = False
MOTOR_RIGHT_INVERT = False

MOTOR_PWM_FREQ_HZ = 1000 

# Line sensors
# GPIO pins from LEFT -> RIGHT
LINE_PINS = [1, 2, 4, 5]

# Line sensor inversion
LINE_INVERT = False 

# --- Loop ---
LOOP_HZ = 50

# --- Line follow (PD) ---
BASE_THROTTLE = 0.20
KP = 0.25
KD = 0.03
MAX_STEER = 0.50
DEADBAND = 0.25

# --- Lost line recovery ---
LINE_LOST_N = 3
SEARCH_STEER = 0.35

# --- Printing ---
PRINT_COOLDOWN_MS = 400

# --- Corner detection ---
CORNER_ERR_THRESH = 0.60     # bigger = only call corners when strongly off-centre
CORNER_N = 2                 # frames in a row required

# --- Junction detection ---
CENTER_THRESH = 0.35         # must be near centered to count as junction
JUNC_N = 3                   # frames in a row required
JUNC_COOLDOWN_MS = 600

# --- Junction behaviour (for test script 2) ---
# "FORCE_STRAIGHT" or "FOLLOW"
JUNC_BEHAVIOR = "FORCE_STRAIGHT"
CROSS_MS = 200               # how long to force straight through junction blob
CROSS_THROTTLE = 0.18        # forward speed during forced crossing

# --- Test 3 intersection action ---
INTERSECTION_ACTION = "STRAIGHT"  # "STRAIGHT", "LEFT", "RIGHT"
CROSS_MS = 200
CROSS_THROTTLE = 0.18

# Intersection detection
CENTER_THRESH = 0.35
JUNC_N = 3
JUNC_COOLDOWN_MS = 600

# Turning
TURN_STEER = 0.45
TURN_TIMEOUT_MS = 1500