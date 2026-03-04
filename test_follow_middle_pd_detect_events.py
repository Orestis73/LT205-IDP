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
    # b01 is list of 0/1 length 4
    return (b01[0] << 3) | (b01[1] << 2) | (b01[2] << 1) | (b01[3] << 0)


def middle_error_white_line(black):
    """
    Line is WHITE, background BLACK.
    black[i]=1 means sensor is on BLACK.
    Use ML/MR only (indices 1,2).

    Returns:
      err: -1,0,+1 or None if lost (both middle on black).
      Convention: err < 0 steer LEFT, err > 0 steer RIGHT
    """
    mL = black[1]
    mR = black[2]

    # centered on white line
    if mL == 0 and mR == 0:
        return 0.0

    # drifted left: left-middle on black -> steer right
    if mL == 1 and mR == 0:
        return +1.0

    # drifted right: right-middle on black -> steer left
    if mL == 0 and mR == 1:
        return -1.0

    # both middle on black -> off the white line
    return None


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

    # --- PD (middle only) ---
    base = getattr(config, "BASE_THROTTLE", 0.20)
    kp = getattr(config, "KP", 0.18)
    kd = getattr(config, "KD", 0.02)
    max_steer = getattr(config, "MAX_STEER", 0.50)

    min_throttle = getattr(config, "MIN_THROTTLE", 0.10)
    slow_k = getattr(config, "SLOW_K", 0.25)

    # --- Lost/search (still middle only) ---
    lost_n = getattr(config, "LINE_LOST_N", 2)
    search_steer = getattr(config, "SEARCH_STEER", 0.45)
    search_throttle = getattr(config, "SEARCH_THROTTLE", 0.0)

    # --- Printing ---
    print_cooldown = getattr(config, "PRINT_COOLDOWN_MS", 200)
    last_print = 0

    # =========================
    # Robust event detection (junctions + corners)
    # =========================
    # Intersection zone: sum(white) >= 3 for N frames
    INTER_SUMW = getattr(config, "INTER_SUMW", 3)
    INTER_ENTER_N = getattr(config, "INTER_ENTER_N", 2)
    INTER_EXIT_N = getattr(config, "INTER_EXIT_N", 2)

    # After leaving the zone, probe straight for this long
    PROBE_MS = getattr(config, "PROBE_MS", 300)

    # Corner hint: middle lost + exactly one outer sees white for N frames
    CORNER_N = getattr(config, "CORNER_N", 2)
    # Only accept corner hint if we were on the line recently (ms)
    RECENT_ONLINE_MS = getattr(config, "RECENT_ONLINE_MS", 400)
    EVENT_COOLDOWN_MS = getattr(config, "EVENT_COOLDOWN_MS", 600)

    in_inter = False
    enter_count = 0
    exit_count = 0

    seen_left = False
    seen_right = False

    probe_active = False
    probe_t0 = 0
    probe_straight_seen = False

    corner_count = 0
    corner_dir = 0  # -1 left, +1 right

    last_event = 0
    t_last_on_line = ticks_ms()

    last_err = 0.0
    lost_count = 0

    t_last = ticks_ms()

    while True:
        # fixed rate loop
        t_now = ticks_ms()
        dt = ticks_diff(t_now, t_last)
        if dt < period_ms:
            sleep_ms(period_ms - dt)
            t_now = ticks_ms()
        t_last = t_now

        black = sensors.read_black()             # 1=black (background), 0=white (line)
        bits_black = sensors.read_bits()         # from hw class (black bits)
        white = [1 - b for b in black]           # 1=white (line), 0=black
        bits_white = bits4_from_list(white)
        sumw = white[0] + white[1] + white[2] + white[3]

        # Track "recently on the line" (any middle sees white)
        if white[1] or white[2]:
            t_last_on_line = t_now

        # -----------------------------
        # 1) INTERSECTION ZONE DETECTOR
        # -----------------------------
        inter_cond = (sumw >= INTER_SUMW)

        if not in_inter and not probe_active:
            if inter_cond:
                enter_count += 1
                if enter_count >= INTER_ENTER_N and ticks_diff(t_now, last_event) > EVENT_COOLDOWN_MS:
                    in_inter = True
                    exit_count = 0
                    enter_count = 0
                    seen_left = False
                    seen_right = False
            else:
                enter_count = 0

        if in_inter:
            # accumulate which branches exist while we're inside the zone
            seen_left |= bool(white[0])
            seen_right |= bool(white[3])

            if not inter_cond:
                exit_count += 1
                if exit_count >= INTER_EXIT_N:
                    in_inter = False
                    exit_count = 0

                    # start probe phase to distinguish T vs +
                    probe_active = True
                    probe_t0 = t_now
                    probe_straight_seen = False
            else:
                exit_count = 0

        if probe_active:
            # If middle sees white during probe window, straight exists
            if white[1] or white[2]:
                probe_straight_seen = True

            if ticks_diff(t_now, probe_t0) >= PROBE_MS:
                probe_active = False

                # classify event
                if seen_left and seen_right:
                    ev = "CROSS" if probe_straight_seen else "T_JUNCTION"   # cases 4 vs 3
                elif seen_left and not seen_right:
                    ev = "LEFT_BRANCH"                                      # case 1
                elif seen_right and not seen_left:
                    ev = "RIGHT_BRANCH"                                     # case 2
                else:
                    ev = "INTERSECTION"                                     # fallback

                print(ev,
                      "white_bits=", "{:04b}".format(bits_white),
                      "black_bits=", bits4(bits_black),
                      "white=", white)

                last_event = t_now

        # -----------------------------
        # 2) CORNER HINT DETECTOR (cases 5-6)
        # -----------------------------
        middle_lost = (white[1] == 0 and white[2] == 0)
        one_outer = (white[0] ^ white[3]) == 1

        if middle_lost and one_outer and not in_inter and not probe_active:
            # only trust corner if we were on-line recently (prevents random lost noise)
            if ticks_diff(t_now, t_last_on_line) <= RECENT_ONLINE_MS and ticks_diff(t_now, last_event) > EVENT_COOLDOWN_MS:
                # require N consecutive frames
                this_dir = -1 if white[0] else +1  # left outer white -> left turn, right outer white -> right turn
                if corner_dir == this_dir:
                    corner_count += 1
                else:
                    corner_dir = this_dir
                    corner_count = 1

                if corner_count >= CORNER_N:
                    ev = "CORNER_LEFT_HINT" if corner_dir < 0 else "CORNER_RIGHT_HINT"
                    print(ev,
                          "white_bits=", "{:04b}".format(bits_white),
                          "black_bits=", bits4(bits_black),
                          "white=", white)
                    last_event = t_now
                    corner_count = 0
        else:
            corner_count = 0

        # -----------------------------
        # Middle-only PD FOLLOWING (unchanged behaviour)
        # -----------------------------
        err = middle_error_white_line(black)

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
                steer = search_steer if last_err >= 0 else -search_steer
                motors.arcade(search_throttle, steer)
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