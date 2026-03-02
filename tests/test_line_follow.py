# tests/test_line_follow_p.py
from utime import ticks_ms, ticks_diff, sleep_ms

import config
from hw.line import LineSensors
from control.line_error import error_from_black
from hw.motors import DCMotor, MotorPair

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def main():
    # --- Hardware init ---
    sensors = LineSensors(config.LINE_PINS, invert=getattr(config, "LINE_INVERT", False))

    left = DCMotor(config.MOTOR_L_DIR, config.MOTOR_L_PWM, pwm_freq_hz=1000, invert=getattr(config, "MOTOR_L_INVERT", True))
    right = DCMotor(config.MOTOR_R_DIR, config.MOTOR_R_PWM, pwm_freq_hz=1000, invert=getattr(config, "MOTOR_R_INVERT", True))
    motors = MotorPair(left, right)

    # --- Control params ---
    loop_hz = getattr(config, "LOOP_HZ", 50)
    period_ms = int(1000 / loop_hz)

    base = getattr(config, "BASE_THROTTLE", 0.25)
    kp = getattr(config, "KP", 0.35)
    max_steer = getattr(config, "MAX_STEER", 0.7)

    lost_n = getattr(config, "LINE_LOST_N", 3)
    search_steer = getattr(config, "SEARCH_STEER", 0.35)

    last_err = 0.0
    lost_count = 0

    t_last = ticks_ms()

    while True:
        # --- fixed-rate loop ---
        t_now = ticks_ms()
        dt = ticks_diff(t_now, t_last)
        if dt < period_ms:
            sleep_ms(period_ms - dt)
            t_now = ticks_ms()
        t_last = t_now

        # --- read sensors + compute error ---
        black = sensors.read_black()          # [L, ML, MR, R], 1=black
        err = error_from_black(black)         # None if line lost

        if err is None:
            lost_count += 1

            # For the first couple of "lost" frames, stop (avoids reacting to a single glitch)
            if lost_count < lost_n:
                motors.arcade(0.0, 0.0)
            else:
                # Search: spin toward last known error direction
                # last_err > 0 means line was on the right -> keep searching right
                steer = search_steer if last_err >= 0 else -search_steer
                motors.arcade(0.0, steer)
            continue

        # line found
        lost_count = 0

        # --- P control ---
        steer = kp * err
        steer = clamp(steer, -max_steer, +max_steer)

        motors.arcade(base, steer)

        last_err = err

main()