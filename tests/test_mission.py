# tests/test_mission_route.py
from utime import ticks_ms, ticks_diff, sleep_ms

import config
from hw.line import LineSensors
from hw.motors import DCMotor, MotorPair


# ---------------- Small helpers ----------------
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def b4(v01):
    # list of 0/1 -> "0101"
    return "{}{}{}{}".format(v01[0], v01[1], v01[2], v01[3])


def middle_error_white_line(black):
    """
    WHITE line on BLACK floor.
    black[i]=1 means sensor sees BLACK (background), 0 means WHITE (line).

    Use ONLY middle sensors:
      ML index 1, MR index 2.

    Returns:
      err = 0   when centered (both middle see white)
      err = +1  when left-middle sees black (drifted left) => steer RIGHT
      err = -1  when right-middle sees black (drifted right) => steer LEFT
      None      when both middle see black (lost)
    """
    mL = black[1]
    mR = black[2]

    if mL == 0 and mR == 0:
        return 0.0
    if mL == 1 and mR == 0:
        return +1.0
    if mL == 0 and mR == 1:
        return -1.0
    return None


def build_mission():
    # (case_id, action)  action: "straight" | "left" | "right"
    m = []
    m.append((3, "right"))

    m.append((2, "straight"))
    m.append((3, "left"))

    for _ in range(6):
        m.append((1, "straight"))

    m.append((4, "straight"))
    m.append((6, "left"))
    m.append((1, "straight"))
    m.append((6, "left"))

    for _ in range(6):
        m.append((1, "straight"))

    m.append((1, "left"))
    m.append((2, "straight"))
    m.append((2, "left"))
    return m


# ---------------- Debug + state labels ----------------
START_BOX = 0
BORDER_PUSH = 1
ACQUIRE = 2
GATE_CLEAR = 3
ARM_WAIT = 4
FOLLOW = 5
IN_INTERSECTION = 6
DO_STRAIGHT = 7
TURN_SPIN = 8
TURN_ALIGN = 9
STOP = 10

STATE_NAMES = {
    START_BOX: "START_BOX",
    BORDER_PUSH: "BORDER_PUSH",
    ACQUIRE: "ACQUIRE",
    GATE_CLEAR: "GATE_CLEAR",
    ARM_WAIT: "ARM_WAIT",
    FOLLOW: "FOLLOW",
    IN_INTERSECTION: "IN_INTERSECTION",
    DO_STRAIGHT: "DO_STRAIGHT",
    TURN_SPIN: "TURN_SPIN",
    TURN_ALIGN: "TURN_ALIGN",
    STOP: "STOP",
}


def main():
    # ---------- Config / debug ----------
    dbg = getattr(config, "DEBUG", True)
    dbg_ms = getattr(config, "DEBUG_MS", 200)
    dbg_t0 = ticks_ms()

    sensors = LineSensors(config.LINE_PINS, invert=getattr(config, "LINE_INVERT", False))

    left = DCMotor(config.MOTOR_L_DIR, config.MOTOR_L_PWM,
                   pwm_freq_hz=getattr(config, "MOTOR_PWM_FREQ_HZ", 1000),
                   invert=getattr(config, "MOTOR_L_INVERT", True))
    right = DCMotor(config.MOTOR_R_DIR, config.MOTOR_R_PWM,
                    pwm_freq_hz=getattr(config, "MOTOR_PWM_FREQ_HZ", 1000),
                    invert=getattr(config, "MOTOR_R_INVERT", True))
    motors = MotorPair(left, right)

    # ---------- Loop timing ----------
    loop_hz = getattr(config, "LOOP_HZ", 50)
    period_ms = int(1000 / loop_hz)
    dt_s = period_ms / 1000.0

    # ---------- PD (your tuned values) ----------
    base = getattr(config, "BASE_THROTTLE", 0.70)
    kp = getattr(config, "KP", 0.20)
    kd = getattr(config, "KD", 0.005)
    max_steer = getattr(config, "MAX_STEER", 0.60)
    min_throttle = getattr(config, "MIN_THROTTLE", 0.10)
    slow_k = getattr(config, "SLOW_K", 0.10)

    # ---------- Start / border / acquire ----------
    start_throttle = getattr(config, "START_THROTTLE", 0.55)
    start_see_white_n = getattr(config, "START_SEE_WHITE_N", 2)

    border_throttle = getattr(config, "BORDER_THROTTLE", 0.35)
    border_exit_n = getattr(config, "BORDER_EXIT_N", 4)
    border_timeout_ms = getattr(config, "BORDER_TIMEOUT_MS", 5000)

    acquire_throttle = getattr(config, "ACQUIRE_THROTTLE", 0.18)
    acquire_steer = getattr(config, "ACQUIRE_STEER", 0.55)
    acquire_good_n = getattr(config, "ACQUIRE_GOOD_N", 3)
    acquire_timeout_ms = getattr(config, "ACQUIRE_TIMEOUT_MS", 5000)
    acquire_search_steer = getattr(config, "ACQUIRE_SEARCH_STEER", 0.55)

    gate_clear_ms = getattr(config, "GATE_CLEAR_MS", 300)
    arm_delay_ms = getattr(config, "ARM_DELAY_MS", 150)

    # ---------- Detection ----------
    inter_sumw = getattr(config, "INTER_SUMW", 3)
    inter_enter_n = getattr(config, "INTER_ENTER_N", 2)
    inter_exit_n = getattr(config, "INTER_EXIT_N", 2)

    corner_n = getattr(config, "CORNER_N", 2)
    recent_good_line_ms = getattr(config, "RECENT_GOOD_LINE_MS", 700)
    corner_dir_sign = getattr(config, "CORNER_DIR_SIGN", 1)

    event_rearm_n = getattr(config, "EVENT_REARM_N", 3)
    event_cooldown_ms = getattr(config, "EVENT_COOLDOWN_MS", 700)

    # ---------- Forced straight ----------
    straight_throttle = getattr(config, "STRAIGHT_THROTTLE", 0.30)
    straight_kp = getattr(config, "STRAIGHT_KP", 0.10)
    straight_max_steer = getattr(config, "STRAIGHT_MAX_STEER", 0.18)
    straight_min_ms = getattr(config, "STRAIGHT_MIN_MS", 250)
    straight_timeout_ms = getattr(config, "STRAIGHT_TIMEOUT_MS", 1200)

    # ---------- Forced turn ----------
    turn_throttle = getattr(config, "TURN_THROTTLE", 0.20)
    turn_steer = getattr(config, "TURN_STEER", 0.85)
    turn_min_ms = getattr(config, "TURN_MIN_MS", 250)
    turn_timeout_ms = getattr(config, "TURN_TIMEOUT_MS", 3500)

    align_throttle = getattr(config, "ALIGN_THROTTLE", 0.22)
    align_timeout_ms = getattr(config, "ALIGN_TIMEOUT_MS", 2500)
    reacquire_n = getattr(config, "REACQUIRE_N", 6)

    # ---------- Lost/search ----------
    search_steer = getattr(config, "SEARCH_STEER", 0.45)
    search_throttle = getattr(config, "SEARCH_THROTTLE", 0.0)

    # ---------- Safety ----------
    stop_on_mismatch = getattr(config, "STOP_ON_MISMATCH", False)

    # ---------- Mission ----------
    mission = build_mission()
    step = 0

    def expected_case():
        return mission[step][0] if step < len(mission) else None

    def expected_action():
        return mission[step][1] if step < len(mission) else None

    # ---------- Runtime memory ----------
    state = START_BOX
    state_t0 = ticks_ms()

    last_event = 0
    events_armed = False

    last_err = 0.0
    last_search_dir = +1

    see_white_count = 0
    border_ok = 0
    acquire_ok = 0
    good_rearm = 0

    enter_count = 0
    exit_count = 0
    seen_left = False
    seen_right = False
    seen_straight = False

    corner_count = 0
    corner_dir = 0

    straight_out = 0

    dir_turn = +1
    t_last_good_line = ticks_ms()

    def set_state(new_state, why=""):
        nonlocal state, state_t0
        state = new_state
        state_t0 = ticks_ms()
        if dbg:
            if why:
                print("STATE ->", STATE_NAMES.get(state, state), "|", why)
            else:
                print("STATE ->", STATE_NAMES.get(state, state))

    print("BOOT: mission length =", len(mission))
    if dbg:
        print("STATE ->", STATE_NAMES.get(state, state))

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
        white = [1 - b for b in black]   # 1 = WHITE
        sumw = white[0] + white[1] + white[2] + white[3]
        err = middle_error_white_line(black)

        good_line = (white[0] == 0 and white[1] == 1 and white[2] == 1 and white[3] == 0)
        if good_line:
            t_last_good_line = t_now

        # intersection condition (robust on fat patches)
        inter_cond = (sumw >= inter_sumw) or (white[0] and white[3]) or ((white[0] or white[3]) and (white[1] or white[2]))

        # debug tick
        if dbg and ticks_diff(t_now, dbg_t0) >= dbg_ms:
            dbg_t0 = t_now
            print("DBG",
                  "S=", STATE_NAMES.get(state, state),
                  "step=", step,
                  "sumw=", sumw,
                  "w=", b4(white),
                  "err=", err,
                  "good=", 1 if good_line else 0,
                  "inter=", 1 if inter_cond else 0,
                  "armed=", 1 if events_armed else 0,
                  "rearm=", good_rearm,
                  "cool=", ticks_diff(t_now, last_event))

        # ---------------- STOP ----------------
        if state == STOP:
            motors.arcade(0.0, 0.0)
            continue

        # ---------------- START_BOX ----------------
        if state == START_BOX:
            motors.arcade(start_throttle, 0.0)

            if sumw >= 1:
                see_white_count += 1
                if see_white_count >= start_see_white_n:
                    see_white_count = 0
                    border_ok = 0
                    set_state(BORDER_PUSH, "saw white at start, pushing border")
            else:
                see_white_count = 0
            continue

        # ---------------- BORDER_PUSH ----------------
        if state == BORDER_PUSH:
            if ticks_diff(t_now, state_t0) >= border_timeout_ms:
                set_state(STOP, "BORDER TIMEOUT (raise BORDER_THROTTLE)")
                continue

            motors.arcade(border_throttle, 0.0)

            if sumw <= 2:
                border_ok += 1
                if border_ok >= border_exit_n:
                    border_ok = 0
                    acquire_ok = 0
                    set_state(ACQUIRE, "border cleared, acquiring clean line")
            else:
                border_ok = 0
            continue

        # ---------------- ACQUIRE ----------------
        if state == ACQUIRE:
            if ticks_diff(t_now, state_t0) >= acquire_timeout_ms:
                set_state(STOP, "ACQUIRE TIMEOUT (raise ACQUIRE_THROTTLE/STEER)")
                continue

            # If we returned to a fat patch, push again
            if sumw >= 3:
                border_ok = 0
                set_state(BORDER_PUSH, "wide white returned during acquire")
                continue

            # If nothing is white, spin to find it
            if sumw == 0:
                motors.arcade(0.0, last_search_dir * acquire_search_steer)
                continue

            # steer toward white using all sensors
            w = (-2 * white[0]) + (-1 * white[1]) + (1 * white[2]) + (2 * white[3])
            if w < 0:
                steer = -acquire_steer
                last_search_dir = -1
            elif w > 0:
                steer = +acquire_steer
                last_search_dir = +1
            else:
                steer = 0.0

            motors.arcade(acquire_throttle, steer)

            if good_line:
                acquire_ok += 1
                if acquire_ok >= acquire_good_n:
                    set_state(GATE_CLEAR, "acquired good line, gate clear")
            else:
                acquire_ok = 0
            continue

        # ---------------- GATE_CLEAR ----------------
        if state == GATE_CLEAR:
            motors.arcade(border_throttle, 0.0)
            if ticks_diff(t_now, state_t0) >= gate_clear_ms:
                events_armed = False
                set_state(ARM_WAIT, "gate cleared, arm wait")
            continue

        # ---------------- ARM_WAIT ----------------
        if state == ARM_WAIT:
            motors.arcade(border_throttle, 0.0)
            if ticks_diff(t_now, state_t0) >= arm_delay_ms:
                events_armed = True
                last_event = t_now
                good_rearm = 0
                enter_count = 0
                corner_count = 0
                print("ARMED. step=0 expected_case=", expected_case())
                set_state(FOLLOW, "begin follow")
            continue

        # ---------------- DO_STRAIGHT ----------------
        if state == DO_STRAIGHT:
            elapsed = ticks_diff(t_now, state_t0)

            steer = 0.0
            if err is not None:
                steer = clamp(straight_kp * err, -straight_max_steer, +straight_max_steer)
            motors.arcade(straight_throttle, steer)

            if (not inter_cond) and (sumw <= 2):
                straight_out += 1
            else:
                straight_out = 0

            if elapsed >= straight_min_ms and straight_out >= inter_exit_n and good_line:
                print("DONE straight. step=", step)
                step += 1
                last_event = t_now
                good_rearm = 0
                set_state(FOLLOW, "straight complete")
                continue

            if elapsed >= straight_timeout_ms:
                last_event = t_now
                good_rearm = 0
                set_state(FOLLOW, "straight timeout -> follow")
            continue

        # ---------------- TURN_SPIN ----------------
        if state == TURN_SPIN:
            elapsed = ticks_diff(t_now, state_t0)
            if elapsed >= turn_timeout_ms:
                set_state(STOP, "TURN TIMEOUT step={}".format(step))
                continue

            motors.arcade(turn_throttle, dir_turn * turn_steer)

            if elapsed < turn_min_ms:
                continue

            # sight new line on turning side
            if dir_turn > 0:
                sighted = (white[2] == 1) or (white[3] == 1)
            else:
                sighted = (white[0] == 1) or (white[1] == 1)

            if sighted:
                acquire_ok = 0
                set_state(TURN_ALIGN, "sighted new branch, aligning")
            continue

        # ---------------- TURN_ALIGN ----------------
        if state == TURN_ALIGN:
            if ticks_diff(t_now, state_t0) >= align_timeout_ms:
                set_state(STOP, "ALIGN TIMEOUT step={}".format(step))
                continue

            if err is None:
                motors.arcade(align_throttle, dir_turn * 0.35)
            else:
                motors.arcade(align_throttle, clamp(0.20 * err, -0.35, +0.35))

            # finish only when we're back on a clean single line (not on fat patch)
            if good_line and (not inter_cond) and (sumw <= 2):
                acquire_ok += 1
                if acquire_ok >= reacquire_n:
                    print("DONE turn. step=", step, "dir=", ("R" if dir_turn > 0 else "L"))
                    step += 1
                    last_event = t_now
                    good_rearm = 0
                    set_state(FOLLOW, "turn complete")
            else:
                acquire_ok = 0
            continue

        # ---------------- IN_INTERSECTION ----------------
        if state == IN_INTERSECTION:
            steer = 0.0
            if err is not None:
                steer = clamp(straight_kp * err, -straight_max_steer, +straight_max_steer)
            motors.arcade(straight_throttle, steer)

            seen_left |= bool(white[0])
            seen_right |= bool(white[3])
            seen_straight |= bool(white[1] or white[2])

            if not inter_cond:
                exit_count += 1
                if exit_count >= inter_exit_n:
                    # classify
                    if seen_left and not seen_right:
                        detected = 1
                    elif seen_right and not seen_left:
                        detected = 2
                    elif seen_left and seen_right:
                        detected = 4 if seen_straight else 3
                    else:
                        detected = 0

                    exp = expected_case()
                    act = expected_action()

                    print("INTER detected=", detected, "expected=", exp, "step=", step,
                          "seenL/R/S=", int(seen_left), int(seen_right), int(seen_straight))

                    if exp is None:
                        set_state(STOP, "mission complete")
                        continue

                    if (detected != exp) and stop_on_mismatch:
                        set_state(STOP, "MISMATCH expected {} got {}".format(exp, detected))
                        continue

                    last_event = t_now
                    good_rearm = 0

                    if act == "straight":
                        straight_out = 0
                        set_state(DO_STRAIGHT, "execute straight")
                        continue
                    if act == "left":
                        dir_turn = -1
                        set_state(TURN_SPIN, "execute left turn")
                        continue
                    if act == "right":
                        dir_turn = +1
                        set_state(TURN_SPIN, "execute right turn")
                        continue

                    set_state(STOP, "bad action {}".format(act))
            else:
                exit_count = 0
            continue

        # ---------------- FOLLOW ----------------
        if state == FOLLOW:
            if step >= len(mission):
                set_state(STOP, "mission complete")
                continue

            # update last_search_dir memory
            if err is not None:
                last_err = err
                if err > 0:
                    last_search_dir = +1
                elif err < 0:
                    last_search_dir = -1

            # rearm counter for event detection
            if good_line:
                if good_rearm < 50:
                    good_rearm += 1
            else:
                if good_rearm > 0:
                    good_rearm -= 1

            can_detect = events_armed and (ticks_diff(t_now, last_event) >= event_cooldown_ms) and (good_rearm >= event_rearm_n)

            if can_detect:
                # --- corner detector ---
                recent_good = ticks_diff(t_now, t_last_good_line) <= recent_good_line_ms
                middle_lost = (white[1] == 0 and white[2] == 0)
                one_outer = (white[0] ^ white[3]) == 1

                if middle_lost and one_outer and recent_good:
                    this_dir = (-1 if white[0] else +1) * corner_dir_sign
                    if corner_dir == this_dir:
                        corner_count += 1
                    else:
                        corner_dir = this_dir
                        corner_count = 1

                    if corner_count >= corner_n:
                        corner_count = 0
                        detected = 6 if corner_dir < 0 else 5
                        exp = expected_case()
                        act = expected_action()

                        print("CORNER detected=", detected, "expected=", exp, "act=", act, "step=", step, "w=", b4(white))

                        last_event = t_now
                        good_rearm = 0

                        if (exp is not None) and (detected != exp) and stop_on_mismatch:
                            set_state(STOP, "corner mismatch expected {} got {}".format(exp, detected))
                            continue

                        if act == "left":
                            dir_turn = -1
                            set_state(TURN_SPIN, "corner->left")
                            continue
                        if act == "right":
                            dir_turn = +1
                            set_state(TURN_SPIN, "corner->right")
                            continue

                        set_state(STOP, "corner bad act {}".format(act))
                        continue
                else:
                    corner_count = 0

                # --- intersection entry ---
                if inter_cond:
                    enter_count += 1
                    if enter_count >= inter_enter_n:
                        enter_count = 0
                        exit_count = 0
                        seen_left = False
                        seen_right = False
                        seen_straight = False
                        last_event = t_now
                        good_rearm = 0
                        set_state(IN_INTERSECTION, "entered inter_cond sumw={} w={}".format(sumw, b4(white)))
                        continue
                else:
                    enter_count = 0

            # lost handling: never freeze
            if err is None:
                if inter_cond or sumw >= 3:
                    motors.arcade(straight_throttle, 0.0)
                else:
                    motors.arcade(search_throttle, last_search_dir * search_steer)
                continue

            # normal PD
            derr = (err - last_err) / dt_s
            steer = kp * err + kd * derr
            steer = clamp(steer, -max_steer, +max_steer)

            throttle = base - slow_k * abs(steer)
            throttle = clamp(throttle, min_throttle, base)

            motors.arcade(throttle, steer)
            last_err = err
            continue


main()