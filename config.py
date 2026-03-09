# config.py

# ---- Motors ----
MOTOR_L_DIR  = 4
MOTOR_L_PWM  = 5
MOTOR_R_DIR  = 7
MOTOR_R_PWM  = 6
MOTOR_L_INVERT = True
MOTOR_R_INVERT = True
MOTOR_PWM_FREQ_HZ = 1000

# ---- Line sensors (LEFT -> RIGHT) ----
LINE_PINS = [0, 1, 2, 3]
LINE_INVERT = False  # keep unless your black/white polarity is wrong

# ---- Loop ----
LOOP_HZ = 50

# ---- PD following (YOUR tuned values) ----
BASE_THROTTLE = 0.70
KP = 0.20
KD = 0.005
MAX_STEER = 0.60
SLOW_K = 0.10
MIN_THROTTLE = 0.10

# ---- Start / border / acquire ----
START_THROTTLE = 0.55
START_SEE_WHITE_N = 2

# This MUST be high enough to move even if you see 1111 at the border.
BORDER_THROTTLE = 0.45
BORDER_EXIT_N = 4
BORDER_TIMEOUT_MS = 5000

# Acquire should also be high enough to move; if motors are weak, raise to 0.22–0.28.
ACQUIRE_THROTTLE = 0.28
ACQUIRE_STEER = 0.55
ACQUIRE_GOOD_N = 3
ACQUIRE_TIMEOUT_MS = 5000
ACQUIRE_SEARCH_STEER = 0.55

GATE_CLEAR_MS = 300
ARM_DELAY_MS = 150

# ---- Detection ----
INTER_SUMW = 3
INTER_ENTER_N = 2
INTER_EXIT_N = 2

CORNER_N = 2
RECENT_GOOD_LINE_MS = 700
CORNER_DIR_SIGN = 1  # set -1 if left/right corner is flipped

# Event detection allowed only after N frames of clean line (prevents noise)
EVENT_REARM_N = 3
EVENT_COOLDOWN_MS = 400

# ---- Forced straight (through intersections) ----
STRAIGHT_THROTTLE = 0.30
STRAIGHT_KP = 0.10
STRAIGHT_MAX_STEER = 0.18
STRAIGHT_MIN_MS = 250
STRAIGHT_TIMEOUT_MS = 1200

# ---- Turning (2-phase: spin->align) ----
# If turning still fails, raise TURN_THROTTLE first, then TURN_STEER, then timeouts.
TURN_THROTTLE = 0.20
TURN_STEER = 0.85
TURN_MIN_MS = 250
TURN_TIMEOUT_MS = 3500

ALIGN_THROTTLE = 0.22
ALIGN_TIMEOUT_MS = 2500
REACQUIRE_N = 6

# ---- Lost/search (only used when middle sensors lose line) ----
SEARCH_THROTTLE = 0.0
SEARCH_STEER = 0.45

# ---- Safety ----
# For debugging set True; for actually running the course set False.
STOP_ON_MISMATCH = False

EVENT_ENTER_N = 3     # debounce for event trigger
TURN_APPROACH_MS = 220   # only used when the event looks like an intersection

# ---- Missing constants required by mission_runner.py ----
INTER_LATCH_MS = 350

FINAL_FOLLOW_MS = 1300
FINAL_FOLLOW_THROTTLE = 0.35
FINAL_FORWARD_MS = 1300
FINAL_FORWARD_THROTTLE = 0.35

# If using biased BORDER_PUSH:
BORDER_BIAS_K = 0.18
BORDER_BIAS_MAX = 0.35

# If using START seek:
START_SEEK_AFTER_MS = 800
START_SEEK_STEER = 0.35

DEBUG=False
DEBUG_MS = 250


SPIN180_THROTTLE = 0.22
SPIN180_STEER = 0.90
SPIN180_MIN_MS = 550
SPIN180_TIMEOUT_MS = 2200

SPIN180_ALIGN_THROTTLE = 0.22
SPIN180_ALIGN_KP = 0.20
SPIN180_ALIGN_MAX_STEER = 0.35
SPIN180_ALIGN_TIMEOUT_MS = 2500
SPIN180_REACQUIRE_N = 6