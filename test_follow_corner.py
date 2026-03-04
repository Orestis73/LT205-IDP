from utime import ticks_ms, ticks_diff, sleep_ms

import config
from hw.line import LineSensors
from hw.motors import DCMotor, MotorPair


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def bits4(x):
    x &= 0xF
    s = ""
    for k in (3, 2, 1, 0):
        s += "1" if (x & (1 << k)) else "0"
    return s


def bits4_from_list(b01):
    return (b01[0] << 3) | (b01[1] << 2) | (b01[2] << 1) | (b01[3] << 0)


def middle_error_white_line(black):
    """
    Line is WHITE, background BLACK.
    black[i] = 1 means sensor sees BLACK.
    Use only middle sensors (ML=1, MR=2).

    Returns:
      err < 0 => steer LEFT
      err > 0 => steer RIGHT
      None    => lost (both middles on black)
    """
    mL = black[1]
    mR = black[2]

    # centered: both middle sensors see white line
    if mL == 0 and mR == 0:
        return 0.0

    # drifted left: left-middle is on black -> steer right
    if mL == 1 and mR == 0:
        return +1.0

    # drifted right: right-middle is on black -> steer left
    if mL == 0 and mR == 1:
        return -1.0

    # both on black: off the white line
    return None


def main():
    sensors = LineSensors(config.LINE_PINS, invert=getattr(config, "LINE_INVERT", False))

    left = DCMotor(config.MOTOR_L_DIR, config.MOTOR_L_PWM, pwm_freq_hz=1000,
                   invert=getattr(config, "MOTOR_L_INVERT", True))
    right = DCMotor(config.MOTOR_R_DIR, config.MOTOR_R_PWM, pwm_freq_hz=1000,
                    invert=getattr(config, "MOTOR_R_INVERT", True))
    motors = MotorPair(left, right)

    # ---------------- Timing ----------------
    loop_hz = getattr(config, "LOOP_HZ", 50)
    period_ms = int(1000 / loop_hz)
    dt_s = period_ms / 1000.0

    # ---------------- PD following (middle only) ----------------
    base = getattr(config, "BASE_THROTTLE", 0.20)
    kp = getattr(config, "KP", 0.18)
    kd = getattr(config, "KD", 0.02)
    max_steer = getattr(config, "MAX_STEER", 0.50)

    min_throttle = getattr(config, "MIN_THROTTLE", 0.10)
    slow_k = getattr(config, "SLOW_K", 0.25)

    # ---------------- Lost/search ----------------
    lost_n = getattr(config, "LINE_LOST_N", 2)
    search_steer = getattr(config, "SEARCH_STEER", 0.45)
    search_throttle = getattr(config, "SEARCH_THROTTLE", 0.0)

    # ---------------- Forced behaviours ----------------
    # Force straight through any intersection
    straight_throttle = getattr(config, "STRAIGHT_THROTTLE", base)
    straight_ms = getattr(config, "STRAIGHT_MS", 350)
    straight_kp = getattr(config, "STRAIGHT_KP", 0.12)              # small correction only
    straight_max_steer = getattr(config, "STRAIGHT_MAX_STEER", 0.20)

    # Force turning at corners
    turn_throttle = getattr(config, "TURN_THROTTLE", 0.12)
    turn_steer = getattr(config, "TURN_STEER", 0.70)
    turn_timeout_ms = getattr(config, "TURN_TIMEOUT_MS", 1200)
    reacquire_n = getattr(config, "REACQUIRE_N", 3)

    # ---------------- Robust event detection thresholds ----------------
    # Junction/intersection zone: many sensors see WHITE
    inter_sumw = getattr(config, "INTER_SUMW", 3)            # sum(white) >= 3
    inter_enter_n = getattr(config, "INTER_ENTER_N", 2)      # consecutive frames to trigger

    # Corner hint (cases 5/6): middle lost + exactly one outer sees WHITE
    corner_n = getattr(config, "CORNER_N", 2)
    recent_online_ms = getattr(config, "RECENT_ONLINE_MS", 400)

    # Print / spam control
    print_cooldown = getattr(config, "PRINT_COOLDOWN_MS", 250)
    last_print = 0

    # ---------------- State machine ----------------
    FOLLOW = 0
    GO_STRAIGHT = 1
    TURN = 2
    SEARCH = 3

    state = FOLLOW
    state_t0 = ticks_ms()

    # direction for TURN/SEARCH: -1 left, +1 right
    dir_turn = +1

    last_err = 0.0
    lost_count = 0

    # event detector memory
    inter_count = 0
    corner_count = 0
    corner_dir = 0

    # reacquire counter for TURN
    reacq_count = 0

    # recent “on line”
    t_last_on_line = ticks_ms()

    t_last = ticks_ms()

    while True:
        # fixed-rate loop
        t_now = ticks_ms()
        dt = ticks_diff(t_now, t_last)
        if dt < period_ms:
            sleep_ms(period_ms - dt)
            t_now = ticks_ms()
        t_last = t_now

        black = sensors.read_black()          # 1=black background, 0=white line
        white = [1 - b for b in black]        # 1=white line
        sumw = white[0] + white[1] + white[2] + white[3]
        bits_black = sensors.read_bits()
        bits_white = bits4_from_list(white)

        # track recent contact with line using middle sensors
        if white[1] or white[2]:
            t_last_on_line = t_now

        # compute middle-only error
        err = middle_error_white_line(black)

        # ------------------------------------------------------------
        # STATE: GO_STRAIGHT (junction policy)
        # ------------------------------------------------------------
        if state == GO_STRAIGHT:
            # Commit forward for straight_ms, with only small middle correction.
            # This prevents PD from "choosing" a side branch.
            steer = 0.0
            if err is not None:
                steer = clamp(straight_kp * err, -straight_max_steer, +straight_max_steer)

            motors.arcade(straight_throttle, steer)

            if ticks_diff(t_now, state_t0) >= straight_ms:
                state = FOLLOW
                inter_count = 0
                if ticks_diff(t_now, last_print) > print_cooldown:
                    print("EXIT GO_STRAIGHT",
                          "white_bits=", "{:04b}".format(bits_white),
                          "black_bits=", bits4(bits_black),
                          "white=", white)
                    last_print = t_now
            continue

        # ------------------------------------------------------------
        # STATE: TURN (corner policy)
        # ------------------------------------------------------------
        if state == TURN:
            # Turn until middle sensors reacquire WHITE (err==0.0) stably
            # or timeout.
            if ticks_diff(t_now, state_t0) > turn_timeout_ms:
                state = SEARCH
                state_t0 = t_now
                lost_count = 0
                reacq_count = 0
                if ticks_diff(t_now, last_print) > print_cooldown:
                    print("TURN TIMEOUT -> SEARCH", "dir=", "L" if dir_turn < 0 else "R")
                    last_print = t_now
                continue

            if err == 0.0:
                reacq_count += 1
                if reacq_count >= reacquire_n:
                    state = FOLLOW
                    reacq_count = 0
                    if ticks_diff(t_now, last_print) > print_cooldown:
                        print("REACQUIRED -> FOLLOW",
                              "white_bits=", "{:04b}".format(bits_white),
                              "black_bits=", bits4(bits_black),
                              "white=", white)
                        last_print = t_now
                    continue
            else:
                reacq_count = 0

            motors.arcade(turn_throttle, dir_turn * turn_steer)
            continue

        # ------------------------------------------------------------
        # STATE: SEARCH (only if lost without a corner hint)
        # ------------------------------------------------------------
        if state == SEARCH:
            if err is not None:
                state = FOLLOW
                lost_count = 0
                if ticks_diff(t_now, last_print) > print_cooldown:
                    print("FOUND -> FOLLOW",
                          "white_bits=", "{:04b}".format(bits_white),
                          "black_bits=", bits4(bits_black),
                          "white=", white)
                    last_print = t_now
                continue

            # Use outer whites to bias search direction if available
            if white[0] and not white[3]:
                dir_turn = -1
            elif white[3] and not white[0]:
                dir_turn = +1
            else:
                dir_turn = +1 if last_err >= 0 else -1

            motors.arcade(search_throttle, dir_turn * search_steer)
            continue

        # ------------------------------------------------------------
        # STATE: FOLLOW (normal PD + event triggers)
        # ------------------------------------------------------------

        # 1) Corner detection (cases 5/6) MUST override everything: TAKE IT.
        # corner hint: middle lost AND exactly one outer sees white, stable for corner_n frames
        middle_lost = (white[1] == 0 and white[2] == 0)
        one_outer = (white[0] ^ white[3]) == 1

        if middle_lost and one_outer and ticks_diff(t_now, t_last_on_line) <= recent_online_ms:
            this_dir = -1 if white[0] else +1
            if corner_dir == this_dir:
                corner_count += 1
            else:
                corner_dir = this_dir
                corner_count = 1

            if corner_count >= corner_n:
                dir_turn = corner_dir
                state = TURN
                state_t0 = t_now
                corner_count = 0
                reacq_count = 0
                if ticks_diff(t_now, last_print) > print_cooldown:
                    print("CORNER -> TURN", "dir=", "L" if dir_turn < 0 else "R",
                          "white_bits=", "{:04b}".format(bits_white),
                          "white=", white)
                    last_print = t_now
                continue
        else:
            corner_count = 0

        # 2) Junction detection: if we enter intersection zone (sum white >=3), GO STRAIGHT.
        # We only trigger while FOLLOWing, so it can't spam inside GO_STRAIGHT.
        if sumw >= inter_sumw:
            inter_count += 1
            if inter_count >= inter_enter_n:
                state = GO_STRAIGHT
                state_t0 = t_now
                inter_count = 0
                if ticks_diff(t_now, last_print) > print_cooldown:
                    print("JUNCTION -> GO_STRAIGHT",
                          "white_bits=", "{:04b}".format(bits_white),
                          "black_bits=", bits4(bits_black),
                          "white=", white)
                    last_print = t_now
                continue
        else:
            inter_count = 0

        # 3) Normal middle PD follow (and lost -> SEARCH)
        if err is None:
            lost_count += 1

            if ticks_diff(t_now, last_print) > print_cooldown:
                print("MIDDLE LOST",
                      "white_bits=", "{:04b}".format(bits_white),
                      "white=", white,
                      "last_err=", last_err)
                last_print = t_now

            if lost_count < lost_n:
                motors.arcade(0.0, 0.0)
            else:
                # bias search by last_err
                dir_turn = +1 if last_err >= 0 else -1
                state = SEARCH
                state_t0 = t_now
            continue

        lost_count = 0

        derr = (err - last_err) / dt_s
        steer = kp * err + kd * derr
        steer = clamp(steer, -max_steer, +max_steer)

        throttle = base - slow_k * abs(steer)
        throttle = clamp(throttle, min_throttle, base)

        motors.arcade(throttle, steer)
        last_err = err


main()