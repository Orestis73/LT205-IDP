# tests/test_follow_junction.py
from utime import ticks_ms, ticks_diff, sleep_ms

import config
from hw.line import LineSensors
from control.line_error import error_from_black
from hw.motors import DCMotor, MotorPair

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def bits4(x):
    x &= 0xF
    s = ""
    for k in (3, 2, 1, 0):
        s += "1" if (x & (1 << k)) else "0"
    return s

FOLLOW = 0
CROSS = 1

def main():
    sensors = LineSensors(config.LINE_PINS, invert=getattr(config, "LINE_INVERT", False))

    left = DCMotor(config.MOTOR_L_DIR, config.MOTOR_L_PWM, pwm_freq_hz=1000,
                   invert=getattr(config, "MOTOR_L_INVERT", False))
    right = DCMotor(config.MOTOR_R_DIR, config.MOTOR_R_PWM, pwm_freq_hz=1000,
                    invert=getattr(config, "MOTOR_R_INVERT", False))
    motors = MotorPair(left, right)

    loop_hz = getattr(config, "LOOP_HZ", 50)
    period_ms = int(1000 / loop_hz)
    dt_s = period_ms / 1000.0

    base = getattr(config, "BASE_THROTTLE", 0.20)
    kp = getattr(config, "KP", 0.25)
    kd = getattr(config, "KD", 0.03)
    max_steer = getattr(config, "MAX_STEER", 0.50)
    deadband = getattr(config, "DEADBAND", 0.25)

    lost_n = getattr(config, "LINE_LOST_N", 3)
    search_steer = getattr(config, "SEARCH_STEER", 0.35)

    center_thresh = getattr(config, "CENTER_THRESH", 0.35)
    junc_n = getattr(config, "JUNC_N", 3)
    junc_cooldown_ms = getattr(config, "JUNC_COOLDOWN_MS", 600)

    behavior = getattr(config, "JUNC_BEHAVIOR", "FORCE_STRAIGHT")  # "FORCE_STRAIGHT" or "FOLLOW"
    cross_ms = getattr(config, "CROSS_MS", 200)
    cross_throttle = getattr(config, "CROSS_THROTTLE", base)

    print_cooldown = getattr(config, "PRINT_COOLDOWN_MS", 400)
    last_print = 0

    # state vars
    state = FOLLOW
    cross_end_time = 0

    last_err = 0.0
    lost_count = 0

    junc_count = 0
    last_junc_time = 0

    t_last = ticks_ms()

    while True:
        # fixed-rate loop
        t_now = ticks_ms()
        dt = ticks_diff(t_now, t_last)
        if dt < period_ms:
            sleep_ms(period_ms - dt)
            t_now = ticks_ms()
        t_last = t_now

        black = sensors.read_black()
        bits = sensors.read_bits()
        s = black[0] + black[1] + black[2] + black[3]
        err = error_from_black(black)

        # --- CROSSING mode: force straight for a short window ---
        if state == CROSS:
            if ticks_diff(t_now, cross_end_time) >= 0:
                state = FOLLOW
            else:
                motors.arcade(cross_throttle, 0.0)
                continue

        # --- Lost line recovery ---
        if err is None:
            lost_count += 1
            if lost_count < lost_n:
                motors.arcade(0.0, 0.0)
            else:
                steer = search_steer if last_err >= 0 else -search_steer
                motors.arcade(0.0, steer)
            continue

        lost_count = 0

        # --- Junction detection (gated: wide + centered + persistence + cooldown) ---
        is_centered = abs(err) <= center_thresh

        if ticks_diff(t_now, last_junc_time) > junc_cooldown_ms:
            if (s >= 3) and is_centered:
                junc_count += 1
            else:
                junc_count = 0
        else:
            junc_count = 0

        if junc_count >= junc_n:
            last_junc_time = t_now
            junc_count = 0

            # Print left/right junction presence based on outer sensors
            if ticks_diff(t_now, last_print) > print_cooldown:
                if black[0] == 1:
                    print("LEFT JUNCTION DETECTED", "bits=", bits4(bits), "black=", black)
                if black[3] == 1:
                    print("RIGHT JUNCTION DETECTED", "bits=", bits4(bits), "black=", black)
                last_print = t_now

            if behavior == "FORCE_STRAIGHT":
                # Force straight through the junction blob
                state = CROSS
                cross_end_time = t_now + cross_ms
                motors.arcade(cross_throttle, 0.0)
                continue
            else:
                # FOLLOW: do nothing special; PD will continue normally
                pass

        # --- PD line following ---
        err_used = err
        if deadband > 0 and abs(err_used) < deadband:
            err_used = 0.0

        derr = (err_used - last_err) / dt_s
        steer = kp * err_used + kd * derr
        steer = clamp(steer, -max_steer, +max_steer)

        motors.arcade(base, steer)
        last_err = err_used

main()