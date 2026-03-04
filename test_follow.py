# tests/test_follow_corner.py
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


def main():
    sensors = LineSensors(config.LINE_PINS, invert=getattr(config, "LINE_INVERT", False))

    left = DCMotor(config.MOTOR_L_DIR, config.MOTOR_L_PWM, pwm_freq_hz=1000,
                   invert=getattr(config, "MOTOR_L_INVERT", True))
    right = DCMotor(config.MOTOR_R_DIR, config.MOTOR_R_PWM, pwm_freq_hz=1000,
                    invert=getattr(config, "MOTOR_R_INVERT", True))
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

    corner_err_thresh = getattr(config, "CORNER_ERR_THRESH", 0.60)
    corner_n = getattr(config, "CORNER_N", 2)

    print_cooldown = getattr(config, "PRINT_COOLDOWN_MS", 400)
    last_print = 0

    last_err = 0.0
    lost_count = 0

    # corner candidate counters (print only)
    left_corner_count = 0
    right_corner_count = 0

    # =========================
    # Intersection detection (print only)
    # =========================
    # Candidate condition: s >= 3 (3 or 4 sensors see black)
    inter_s_thresh = getattr(config, "INTERSECTION_S_THRESH", 3)

    # Require N consecutive frames to declare intersection
    inter_n = getattr(config, "INTERSECTION_N", 3)

    # After firing, wait until we "exit" (s <= 2 for M frames) before re-arming
    inter_exit_s = getattr(config, "INTERSECTION_EXIT_S", 2)
    inter_exit_n = getattr(config, "INTERSECTION_EXIT_N", 3)

    # Cooldown to avoid spam if you crawl on the patch
    inter_cooldown_ms = getattr(config, "INTERSECTION_COOLDOWN_MS", 800)

    inter_count = 0
    inter_armed = True
    inter_exit_count = 0
    last_inter = 0

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

        # -------- Lost line recovery (unchanged) --------
        if err is None:
            lost_count += 1

            if ticks_diff(t_now, last_print) > print_cooldown:
                if last_err < 0:
                    print("LEFT CORNER DETECTED (LINE LOST)", "bits=", bits4(bits))
                else:
                    print("RIGHT CORNER DETECTED (LINE LOST)", "bits=", bits4(bits))
                last_print = t_now

            if lost_count < lost_n:
                motors.arcade(0.0, 0.0)
            else:
                steer = search_steer if last_err >= 0 else -search_steer
                motors.arcade(0.0, steer)
            continue

        lost_count = 0

        # =========================
        # Intersection detection (ROBUST, PRINT ONLY)
        # =========================
        if inter_armed:
            if s >= inter_s_thresh:
                inter_count += 1
            else:
                inter_count = 0

            # Fire event if stable and not in cooldown
            if inter_count >= inter_n and ticks_diff(t_now, last_inter) > inter_cooldown_ms:
                # Print once
                print("INTERSECTION DETECTED",
                      "s=", s,
                      "bits=", bits4(bits),
                      "black=", black,
                      "err=", err)
                last_inter = t_now

                # Disarm until we exit the patch
                inter_armed = False
                inter_exit_count = 0
                inter_count = 0
        else:
            # Wait until we leave the intersection region to re-arm
            if s <= inter_exit_s:
                inter_exit_count += 1
                if inter_exit_count >= inter_exit_n:
                    inter_armed = True
                    inter_exit_count = 0
            else:
                inter_exit_count = 0

        # -------- Corner detection while still on line (print only, unchanged) --------
        if s >= 2 and err <= -corner_err_thresh:
            left_corner_count += 1
            right_corner_count = 0
        elif s >= 2 and err >= corner_err_thresh:
            right_corner_count += 1
            left_corner_count = 0
        else:
            left_corner_count = 0
            right_corner_count = 0

        if left_corner_count >= corner_n and ticks_diff(t_now, last_print) > print_cooldown:
            print("LEFT CORNER DETECTED", "bits=", bits4(bits), "black=", black, "err=", err)
            last_print = t_now
            left_corner_count = 0

        if right_corner_count >= corner_n and ticks_diff(t_now, last_print) > print_cooldown:
            print("RIGHT CORNER DETECTED", "bits=", bits4(bits), "black=", black, "err=", err)
            last_print = t_now
            right_corner_count = 0

        # -------- PD line following (unchanged) --------
        err_used = err
        if deadband > 0 and abs(err_used) < deadband:
            err_used = 0.0

        derr = (err_used - last_err) / dt_s
        steer = kp * err_used + kd * derr
        steer = clamp(steer, -max_steer, +max_steer)

        motors.arcade(base, steer)
        last_err = err_used


main()