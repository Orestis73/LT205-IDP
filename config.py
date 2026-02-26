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
LINE_PINS = [10, 11, 12, 13]

# Line sensor inversion
LINE_INVERT = False 

# Distance sensor (VCNL4010 example uses I2C0 on GP17/GP16 @100kHz) fileciteturn9file0
I2C_ID = 0
I2C_SCL_PIN = 17
I2C_SDA_PIN = 16
I2C_FREQ_HZ = 100_000

# -------- Line following tuning --------
# Start conservative. Raise BASE_SPEED slowly after it can follow at crawl speed.
BASE_SPEED = 0.25   # -1..+1 (fraction of full PWM), forward only for line follow
KP = 0.60           # proportional gain
KD = 0.02           # derivative gain (helps reduce oscillation). Set to 0 if noisy.

# Clamp steering correction
U_MAX = 0.60

# Sensor weights for 4 sensors [L, ML, MR, R]
LINE_WEIGHTS = [-1.5, -0.5, 0.5, 1.5]

# Debounce / detection
JUNCTION_SUM_BLACK = 3     # >=3 black sensors indicates junction (typical)
JUNCTION_CONSEC = 3        # consecutive samples to confirm junction
LINE_LOST_CONSEC = 3       # consecutive samples of sum_black==0 to declare "lost"
REACQUIRE_CONSEC = 2       # consecutive samples of center sensors seeing black

# Turn behaviour
TURN_SPEED = 0.25          # rotate-in-place speed during turns
TURN_TIMEOUT_MS = 2000     # safety timeout to avoid spinning forever

# Main loop timing
CONTROL_HZ = 50            # 50Hz control loop is plenty for digital sensors
