from utime import ticks_ms, ticks_diff, sleep_ms

import config
from hw.line import LineSensors
from control.line_error import error_from_black
from hw.motors import DCMotor, MotorPair


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else hi if x > hi else x


def bits4(x):
    x &= 0xF
    s = ""
    for k in (3, 2, 1, 0):
        s += "1" if (x & (1 << k)) else "0"
    return s


LEFT_MASK  = 0b1100
RIGHT_MASK = 0b0011
MID_MASK   = 0b0110


def choose_dir(err, bits, last_err):
    """Return -1 for LEFT, +1 for RIGHT."""
    if err is not None and abs(err) > 0.05:
        return -1 if err < 0 else +1

    left_seen = (bits & LEFT_MASK) != 0
    right_seen = (bits & RIGHT_MASK) != 0

    if left_seen and not right_seen:
        return -1
    if right_seen and not left_seen:
        return +1

    return -1 if last_err < 0 else +1


def is_intersection(bits, s, err):
    # Same detector for corner-tracks and junctions
    if bits == 0b1111:
        return True
    if s >= config.INTERSECTION_S_THRESH:
        return True
    # optionally treat "wide/centered" as intersection-ish
    if s == 2 and err is not None and abs(err) <= config.INTERSECTION_CENTER_ERR:
        return True
    return False


def is_corner_approach(s, err):
    if err is None:
        return False
    return (s >= 2) and (abs(err) >= config.CORNER_ERR_THRESH)


def decide_junction_action(bits, err, last_err):
    """
    Returns: "left" | "right" | "straight"
    Policy is intentionally simple because 4 digital sensors are limited.
    """
    a = getattr(config, "JUNCTION_ACTION", "straight")
    if a in ("left", "right", "straight"):
        return a

    # "auto":
    # If we look centered-ish, prefer straight.
    if err is not None and abs(err) <= config.INTERSECTION_CENTER_ERR and (bits & MID_MASK):
        return "straight"

    # Otherwise turn towards the side we were already biased towards.
    if err is not None:
        return "left" if err < 0 else "right"

    # Fall back to last_err sign or preferred turn
    if abs(last_err) > 0.05:
        return "left" if last_err < 0 else "right"

    pref = getattr(config, "PREFERRED_TURN", "left")
    return "right" if pref == "right" else "left"


def main():
    sensors = LineSensors(config.LINE_PINS, invert=getattr(config, "LINE_INVERT", False))

    left = DCMotor(config.MOTOR_L_DIR, config.MOTOR_L_PWM, pwm_freq_hz=1000,
                   invert=getattr(config, "MOTOR_L_INVERT", False))
    right = DCMotor(config.MOTOR_R_DIR, config.MOTOR_R_PWM, pwm_freq_hz=1000,
                    invert=getattr(config, "MOTOR_R_INVERT", False))
    motors = MotorPair(left, right)

    junction_mode = getattr(config, "JUNCTION_MODE", False)

    loop_hz = getattr(config, "LOOP_HZ", 50)
    period_ms = int(1000 / loop_hz)
    dt_s = period_ms / 1000.0

    base = getattr(config, "BASE_THROTTLE", 0.22)
    min_th = getattr(config, "MIN_THROTTLE", 0.08)
    speed_reduce_k = getattr(config, "SPEED_REDUCE_K", 0.35)

    kp = getattr(config, "KP", 0.28)
    kd = getattr(config, "KD", 0.035)
    max_steer = getattr(config, "MAX_STEER", 0.75)
    deadband = getattr(config, "DEADBAND", 0.18)

    print_cooldown = getattr(config, "PRINT_COOLDOWN_MS", 250)
    last_print = 0

    lost_n = getattr(config, "LINE_LOST_N", 2)
    search_fwd = getattr(config, "SEARCH_FWD", 0.07)
    search_steer = getattr(config, "SEARCH_STEER", 0.55)
    search_timeout_ms = getattr(config, "SEARCH_TIMEOUT_MS", 2000)

    turn_th = getattr(config, "TURN_THROTTLE", 0.12)
    turn_steer = getattr(config, "TURN_STEER", 0.80)
    turn_timeout_ms = getattr(config, "TURN_TIMEOUT_MS", 1300)

    reacq_err = getattr(config, "REACQUIRE_ERR", 0.35)
    reacq_n = getattr(config, "REACQUIRE_N", 3)

    cross_th = getattr(config, "CROSS_THROTTLE", 0.16)
    cross_timeout_ms = getattr(config, "CROSS_TIMEOUT_MS", 600)

    intersection_hold_n = getattr(config, "INTERSECTION_HOLD_N", 2)
    corner_hold_n = getattr(config, "CORNER_HOLD_N", 2)

    FOLLOW, TURN, SEARCH, CROSS = 0, 1, 2, 3
    state = FOLLOW
    state_t0 = ticks_ms()

    last_err = 0.0
    lost_count = 0

    dir_turn = +1
    reacq_count = 0

    inter_count = 0
    left_corner_count = 0
    right_corner_count = 0

    t_last = ticks_ms()

    while True:
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

        # ---------------- FOLLOW ----------------
        if state == FOLLOW:
            # lost handling -> SEARCH (do not stop/spin-in-place)
            if err is None:
                lost_count += 1
                if lost_count >= lost_n:
                    dir_turn = choose_dir(None, bits, last_err)
                    state = SEARCH
                    state_t0 = t_now
                    reacq_count = 0
                    if ticks_diff(t_now, last_print) > print_cooldown:
                        print("LOST -> SEARCH dir=", "L" if dir_turn < 0 else "R", "bits=", bits4(bits))
                        last_print = t_now
                else:
                    motors.arcade(0.0, 0.0)
                continue
            else:
                lost_count = 0

            # intersection detector (held for N frames)
            if is_intersection(bits, s, err):
                inter_count += 1
            else:
                inter_count = 0

            if inter_count >= intersection_hold_n:
                inter_count = 0
                if not junction_mode:
                    # corner-track: commit turn in inferred direction
                    dir_turn = choose_dir(err, bits, last_err)
                    state = TURN
                    state_t0 = t_now
                    reacq_count = 0
                    if ticks_diff(t_now, last_print) > print_cooldown:
                        print("INTERSECTION(corner) -> TURN",
                              "dir=", "L" if dir_turn < 0 else "R",
                              "bits=", bits4(bits), "black=", black, "err=", err)
                        last_print = t_now
                    continue
                else:
                    # junction: choose L/R/STRAIGHT
                    action = decide_junction_action(bits, err, last_err)
                    if action == "straight":
                        state = CROSS
                    else:
                        dir_turn = -1 if action == "left" else +1
                        state = TURN
                    state_t0 = t_now
                    reacq_count = 0
                    if ticks_diff(t_now, last_print) > print_cooldown:
                        print("INTERSECTION(junction) ->", "CROSS" if state == CROSS else "TURN",
                              "action=", action, "bits=", bits4(bits), "black=", black, "err=", err)
                        last_print = t_now
                    continue

            # corner approach detector (pre-emptively commit)
            if is_corner_approach(s, err):
                if err <= -config.CORNER_ERR_THRESH:
                    left_corner_count += 1
                    right_corner_count = 0
                elif err >= config.CORNER_ERR_THRESH:
                    right_corner_count += 1
                    left_corner_count = 0
                else:
                    left_corner_count = 0
                    right_corner_count = 0

                if left_corner_count >= corner_hold_n or right_corner_count >= corner_hold_n:
                    dir_turn = -1 if left_corner_count >= corner_hold_n else +1
                    state = TURN
                    state_t0 = t_now
                    reacq_count = 0
                    left_corner_count = 0
                    right_corner_count = 0
                    if ticks_diff(t_now, last_print) > print_cooldown:
                        print("CORNER APPROACH -> TURN",
                              "dir=", "L" if dir_turn < 0 else "R",
                              "bits=", bits4(bits), "black=", black, "err=", err)
                        last_print = t_now
                    continue
            else:
                left_corner_count = 0
                right_corner_count = 0

            # PD follow with speed reduction on large error
            err_used = err
            if deadband > 0 and abs(err_used) < deadband:
                err_used = 0.0

            derr = (err_used - last_err) / dt_s
            steer = kp * err_used + kd * derr
            steer = clamp(steer, -max_steer, +max_steer)

            throttle = base - speed_reduce_k * abs(err_used)
            throttle = clamp(throttle, min_th, base)

            motors.arcade(throttle, steer)
            last_err = err_used
            continue

        # ---------------- TURN (commit until reacquire) ----------------
        if state == TURN:
            if ticks_diff(t_now, state_t0) > turn_timeout_ms:
                state = SEARCH
                state_t0 = t_now
                if ticks_diff(t_now, last_print) > print_cooldown:
                    print("TURN TIMEOUT -> SEARCH", "dir=", "L" if dir_turn < 0 else "R")
                    last_print = t_now
                continue

            # reacquire: need stable small error for N frames
            if err is not None and s >= 1 and abs(err) <= reacq_err:
                reacq_count += 1
                if reacq_count >= reacq_n:
                    state = FOLLOW
                    reacq_count = 0
                    last_err = err
                    if ticks_diff(t_now, last_print) > print_cooldown:
                        print("REACQUIRED -> FOLLOW", "bits=", bits4(bits), "black=", black, "err=", err)
                        last_print = t_now
                    continue
            else:
                reacq_count = 0

            # IMPORTANT: turn with some forward motion (not spin-in-place)
            motors.arcade(turn_th, dir_turn * turn_steer)

            if err is not None:
                last_err = err
            continue

        # ---------------- SEARCH (lost line) ----------------
        if state == SEARCH:
            if err is not None:
                state = FOLLOW
                last_err = err
                if ticks_diff(t_now, last_print) > print_cooldown:
                    print("FOUND -> FOLLOW", "bits=", bits4(bits), "black=", black, "err=", err)
                    last_print = t_now
                continue

            # flip direction occasionally
            if ticks_diff(t_now, state_t0) > search_timeout_ms:
                dir_turn = -dir_turn
                state_t0 = t_now
                if ticks_diff(t_now, last_print) > print_cooldown:
                    print("SEARCH FLIP dir=", "L" if dir_turn < 0 else "R")
                    last_print = t_now

            motors.arcade(search_fwd, dir_turn * search_steer)
            continue

        # ---------------- CROSS (go straight through junction) ----------------
        if state == CROSS:
            if ticks_diff(t_now, state_t0) > cross_timeout_ms:
                state = FOLLOW
                if ticks_diff(t_now, last_print) > print_cooldown:
                    print("CROSS TIMEOUT -> FOLLOW")
                    last_print = t_now
                continue

            motors.arcade(cross_th, 0.0)

            # If we exit the intersection (not many blacks) and have err, resume FOLLOW
            if err is not None and s <= 2 and not is_intersection(bits, s, err):
                state = FOLLOW
                last_err = err
                if ticks_diff(t_now, last_print) > print_cooldown:
                    print("CROSS EXIT -> FOLLOW", "bits=", bits4(bits), "black=", black, "err=", err)
                    last_print = t_now
            continue


main()