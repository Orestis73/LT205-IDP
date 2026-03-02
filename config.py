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

# Control tuning
LOOP_HZ = 50

BASE_THROTTLE = 0.25     # start low (0.15–0.35)
KP = 0.35                # start here, tune up/down
MAX_STEER = 0.7          # clamp steering so you don’t saturate instantly

# Line lost behaviour
LINE_LOST_N = 3          # how many cycles of "no line" before search
SEARCH_STEER = 0.35      # turn-in-place strength during search