from utime import ticks_ms, ticks_diff, sleep_ms

import config
from control.ulilities import set_state, clamp
from control.pd import middle_error_white_line, pd_follow


period_ms = int(1000 / config.LOOP_HZ)
dt_s = period_ms / 1000.0

DEBUG = getattr(config, "DEBUG_MOTION", True)
DEBUG_EVERY = getattr(config, "DEBUG_EVERY", 5)

# Safe fallbacks so v3 can run before you add dedicated place constants to config.py
PLACE_FORWARD_TIMEOUT_MS = getattr(config, "PLACE_FORWARD_TIMEOUT_MS", config.GRAB_FORWARD_MS)
PLACE_WAIT_MS = getattr(config, "PLACE_WAIT_MS", config.GRAB_WAIT_MS)
PLACE_REVERSE_MS = getattr(config, "PLACE_REVERSE_MS", config.GRAB_FORWARD_MS)
PLACE_FORWARD_THROTTLE = getattr(config, "PLACE_FORWARD_THROTTLE", config.GRAB_FORWARD_THROTTLE)
PLACE_REVERSE_THROTTLE_MAG = abs(
    getattr(config, "PLACE_REVERSE_THROTTLE", abs(config.GRAB_REVERSE_THROTTLE))
)


def fixed_rate_tick(t_last, period_ms):
    t_now = ticks_ms()
    dt = ticks_diff(t_now, t_last)
    if dt < period_ms:
        sleep_ms(period_ms - dt)
        t_now = ticks_ms()
    return t_now, t_now


def _bits(x):
    return "".join("1" if v else "0" for v in x)


def _dbg(
    mem,
    label,
    t_now,
    black,
    white,
    sumw,
    good_line,
    inter_cond,
    corner_raw,
    corner_cond,
    event_cond,
    recent_good,
    err,
    extra="",
):
    if not DEBUG:
        return

    mem.dbg_count = getattr(mem, "dbg_count", 0) + 1
    if mem.dbg_count % DEBUG_EVERY != 0:
        return

    print(
        "DBG",
        "S=", label,
        "step=", getattr(mem, "step", None),
        "sumw=", sumw,
        "b=", _bits(black),
        "w=", _bits(white),
        "err=", err,
        "good=", 1 if good_line else 0,
        "recent=", 1 if recent_good else 0,
        "inter=", 1 if inter_cond else 0,
        "corner_raw=", 1 if corner_raw else 0,
        "corner=", 1 if corner_cond else 0,
        "event=", 1 if event_cond else 0,
        extra,
    )


def compute_event(t_now, mem, corner_raw, inter_cond):
    recent_good = ticks_diff(t_now, mem.t_last_good_line) <= config.RECENT_GOOD_LINE_MS
    corner_cond = recent_good and corner_raw and (not inter_cond)
    event_cond = inter_cond or corner_cond
    return inter_cond, corner_cond, event_cond, recent_good


def border_push_movement(mem, motors, sensors, states):
    t_last = ticks_ms()

    while mem.state != states.FOLLOW and mem.state != states.STOP:
        t_now, t_last = fixed_rate_tick(t_last, period_ms)
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(
            t_now, mem, corner_raw, inter_cond
        )
        err = middle_error_white_line(black)

        if mem.state == states.START_BOX:
            motors.arcade(config.START_THROTTLE, 0.0)

            if sumw >= 1:
                mem.see_white_count += 1
                if mem.see_white_count >= config.START_SEE_WHITE_N:
                    mem.see_white_count = 0
                    mem.border_ok = 0
                    set_state(mem, states.BORDER_PUSH, "saw white at start, pushing border")
            else:
                mem.see_white_count = 0
            continue

        if mem.state == states.BORDER_PUSH:
            elapsed = ticks_diff(t_now, mem.state_t0)
            if elapsed >= config.BORDER_TIMEOUT_MS:
                set_state(mem, states.STOP, "BORDER TIMEOUT")
                continue

            w = (-2 * white[0]) + (-1 * white[1]) + (1 * white[2]) + (2 * white[3])
            if w < 0:
                mem.last_search_dir = -1
            elif w > 0:
                mem.last_search_dir = +1

            steer = clamp(
                config.BORDER_BIAS_K * w,
                -config.BORDER_BIAS_MAX,
                +config.BORDER_BIAS_MAX,
            )
            motors.arcade(config.BORDER_THROTTLE, steer)

            if sumw <= 2:
                mem.border_ok += 1
                if mem.border_ok >= config.BORDER_EXIT_N:
                    mem.border_ok = 0
                    mem.acquire_ok = 0
                    set_state(mem, states.ACQUIRE, "border cleared, acquiring clean line")
            else:
                mem.border_ok = 0
            continue

        if mem.state == states.ACQUIRE:
            elapsed = ticks_diff(t_now, mem.state_t0)
            if elapsed >= config.ACQUIRE_TIMEOUT_MS:
                set_state(mem, states.STOP, "ACQUIRE TIMEOUT")
                continue

            if sumw >= 3:
                mem.border_ok = 0
                set_state(mem, states.BORDER_PUSH, "wide white returned during acquire")
                continue

            if sumw == 0:
                motors.arcade(0.0, mem.last_search_dir * config.ACQUIRE_SEARCH_STEER)
                continue

            w = (-2 * white[0]) + (-1 * white[1]) + (1 * white[2]) + (2 * white[3])
            if w < 0:
                steer = -config.ACQUIRE_STEER
                mem.last_search_dir = -1
            elif w > 0:
                steer = +config.ACQUIRE_STEER
                mem.last_search_dir = +1
            else:
                steer = 0.0

            motors.arcade(config.ACQUIRE_THROTTLE, steer)

            if good_line:
                mem.acquire_ok += 1
                if mem.acquire_ok >= config.ACQUIRE_GOOD_N:
                    set_state(mem, states.GATE_CLEAR, "acquired good line, gate clear")
            else:
                mem.acquire_ok = 0
            continue

        if mem.state == states.GATE_CLEAR:
            motors.arcade(config.BORDER_THROTTLE, 0.0)

            if ticks_diff(t_now, mem.state_t0) >= config.GATE_CLEAR_MS:
                mem.events_armed = True
                set_state(mem, states.FOLLOW, "gate cleared")
            continue


def turning_movement(mem, motors, sensors, states, advance_step=True):
    t_last = ticks_ms()

    while mem.state not in (states.FOLLOW, states.STOP):
        t_now, t_last = fixed_rate_tick(t_last, period_ms)
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(
            t_now, mem, corner_raw, inter_cond
        )
        err = middle_error_white_line(black)

        if mem.state == states.TURN_APPROACH:
            motors.arcade(config.STRAIGHT_THROTTLE, 0.0)

            if ticks_diff(t_now, mem.state_t0) >= config.TURN_APPROACH_MS:
                set_state(mem, states.TURN_SPIN, "approach done -> spin")
            continue

        if mem.state == states.TURN_SPIN:
            elapsed = ticks_diff(t_now, mem.state_t0)
            if elapsed >= config.TURN_TIMEOUT_MS:
                set_state(mem, states.STOP, "TURN TIMEOUT step={}".format(mem.step))
                continue

            motors.arcade(config.TURN_THROTTLE, mem.dir_turn * config.TURN_STEER)

            if elapsed < config.TURN_MIN_MS:
                continue

            if mem.dir_turn > 0:
                sighted = (white[2] == 1) or (white[3] == 1)
            else:
                sighted = (white[0] == 1) or (white[1] == 1)

            if sighted:
                mem.acquire_ok = 0
                set_state(mem, states.TURN_ALIGN, "sighted branch, aligning")
            continue

        if mem.state == states.TURN_ALIGN:
            elapsed = ticks_diff(t_now, mem.state_t0)
            if elapsed >= config.ALIGN_TIMEOUT_MS:
                set_state(mem, states.STOP, "ALIGN TIMEOUT step={}".format(mem.step))
                continue

            if err is None:
                motors.arcade(config.ALIGN_THROTTLE, mem.dir_turn * 0.35)
            else:
                motors.arcade(config.ALIGN_THROTTLE, clamp(0.20 * err, -0.35, +0.35))

            if good_line and (not inter_cond) and (sumw <= 2):
                mem.acquire_ok += 1
                if mem.acquire_ok >= config.REACQUIRE_N:
                    if advance_step:
                        mem.step += 1
                    mem.last_event_t = t_now
                    mem.good_rearm = 0
                    set_state(mem, states.FOLLOW, "turn complete")
            else:
                mem.acquire_ok = 0
            continue

    return mem.state == states.STOP


def straight_movement(mem, motors, sensors, states, mission=None):
    t_last = ticks_ms()

    while mem.state not in (states.FOLLOW, states.STOP):
        t_now, t_last = fixed_rate_tick(t_last, period_ms)
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(
            t_now, mem, corner_raw, inter_cond
        )
        err = middle_error_white_line(black)

        if mem.state == states.DO_STRAIGHT:
            elapsed = ticks_diff(t_now, mem.state_t0)

            steer = 0.0
            if err is not None:
                steer = clamp(
                    config.STRAIGHT_KP * err,
                    -config.STRAIGHT_MAX_STEER,
                    +config.STRAIGHT_MAX_STEER,
                )

            motors.arcade(config.STRAIGHT_THROTTLE, steer)

            if (not inter_cond) and (sumw <= 2):
                mem.straight_out += 1
            else:
                mem.straight_out = 0

            if elapsed >= config.STRAIGHT_MIN_MS and mem.straight_out >= config.INTER_EXIT_N and good_line:
                mem.step += 1
                mem.last_event_t = t_now
                mem.good_rearm = 0
                set_state(mem, states.FOLLOW, "straight complete")
                continue

            if elapsed >= config.STRAIGHT_TIMEOUT_MS:
                mem.last_event_t = t_now
                mem.good_rearm = 0
                set_state(mem, states.STOP, "straight timeout")
                continue


def spin180_movement(mem, motors, sensors, states, advance_step=True):
    t_last = ticks_ms()

    while mem.state not in (states.FOLLOW, states.STOP):
        t_now, t_last = fixed_rate_tick(t_last, period_ms)
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(
            t_now, mem, corner_raw, inter_cond
        )
        err = middle_error_white_line(black)

        if mem.state == states.SPIN180_SPIN:
            elapsed = ticks_diff(t_now, mem.state_t0)
            if elapsed >= config.SPIN180_TIMEOUT_MS:
                set_state(mem, states.STOP, "SPIN180 TIMEOUT")
                continue

            motors.arcade(config.SPIN180_THROTTLE, mem.dir_turn * config.SPIN180_STEER)

            if elapsed < config.SPIN180_MIN_MS:
                continue

            if sumw >= 1:
                mem.acquire_ok = 0
                set_state(mem, states.SPIN180_ALIGN, "180 spin sighted line -> align")
            continue

        if mem.state == states.SPIN180_ALIGN:
            elapsed = ticks_diff(t_now, mem.state_t0)
            if elapsed >= config.SPIN180_ALIGN_TIMEOUT_MS:
                set_state(mem, states.STOP, "SPIN180 ALIGN TIMEOUT")
                continue

            if err is None:
                motors.arcade(config.SPIN180_ALIGN_THROTTLE, mem.dir_turn * 0.35)
            else:
                motors.arcade(
                    config.SPIN180_ALIGN_THROTTLE,
                    clamp(
                        config.SPIN180_ALIGN_KP * err,
                        -config.SPIN180_ALIGN_MAX_STEER,
                        +config.SPIN180_ALIGN_MAX_STEER,
                    ),
                )

            if good_line and (not inter_cond) and (sumw <= 2):
                mem.acquire_ok += 1
                if mem.acquire_ok >= config.SPIN180_REACQUIRE_N:
                    if advance_step:
                        mem.step += 1
                    mem.last_event_t = t_now
                    mem.good_rearm = 0
                    set_state(mem, states.FOLLOW, "180 spin complete")
            else:
                mem.acquire_ok = 0
            continue


def _reverse_follow(err, last_err):
    if err is None:
        return -abs(config.GRAB_REVERSE_THROTTLE), 0.0

    throttle, steer = pd_follow(err, last_err, dt_s)
    return -throttle, -steer


def _do_local_enter_action(mem, motors, sensors, states, enter_action):
    if enter_action == "straight":
        set_state(mem, states.PLACE_FORWARD, "place -> straight into bay")
        return True

    if enter_action == "left":
        mem.dir_turn = -1
        set_state(mem, states.TURN_SPIN, "place -> left into bay")
        turning_movement(mem, motors, sensors, states, advance_step=False)
        return mem.state != states.STOP

    if enter_action == "right":
        mem.dir_turn = +1
        set_state(mem, states.TURN_SPIN, "place -> right into bay")
        turning_movement(mem, motors, sensors, states, advance_step=False)
        return mem.state != states.STOP

    if enter_action == "180":
        mem.dir_turn = +1
        set_state(mem, states.SPIN180_SPIN, "place -> 180 into bay")
        spin180_movement(mem, motors, sensors, states, advance_step=False)
        return mem.state != states.STOP

    set_state(mem, states.STOP, "invalid enter_action {}".format(enter_action))
    return False


def _do_local_exit_action(mem, motors, sensors, states, exit_action, exit_spin_dir):
    if exit_action in (None, "none"):
        return True

    if exit_action == "180":
        mem.dir_turn = exit_spin_dir
        set_state(mem, states.SPIN180_SPIN, "place reverse done -> exit 180")
        spin180_movement(mem, motors, sensors, states, advance_step=False)
        return mem.state != states.STOP

    if exit_action == "left":
        mem.dir_turn = -1
        set_state(mem, states.TURN_SPIN, "place reverse done -> exit left")
        turning_movement(mem, motors, sensors, states, advance_step=False)
        return mem.state != states.STOP

    if exit_action == "right":
        mem.dir_turn = +1
        set_state(mem, states.TURN_SPIN, "place reverse done -> exit right")
        turning_movement(mem, motors, sensors, states, advance_step=False)
        return mem.state != states.STOP

    set_state(mem, states.STOP, "invalid exit_action {}".format(exit_action))
    return False


def grab_movement(mem, motors, sensors, states, task_sensors, stack_name):
    """
    Local grab macro:
        turn into branch
        go forward
        grab
        reverse to the SAME node
        STOP there and return to FOLLOW

    It does NOT turn back onto the corridor.
    """
    t_last = ticks_ms()
    did_close = False

    while mem.state not in (states.FOLLOW, states.STOP):
        t_now, t_last = fixed_rate_tick(t_last, period_ms)
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(
            t_now, mem, corner_raw, inter_cond
        )
        err = middle_error_white_line(black)

        if mem.state == states.GRAB:
            set_state(mem, states.TURN_SPIN, "grab -> enter branch")
            turning_movement(mem, motors, sensors, states, advance_step=False)

            if mem.state == states.STOP:
                continue

            mem.last_err = 0.0 if err is None else err
            set_state(mem, states.GRAB_FORWARD, "grab turn complete -> forward")
            continue

        if mem.state == states.GRAB_FORWARD:
            elapsed = ticks_diff(t_now, mem.state_t0)

            if err is None:
                motors.arcade(config.GRAB_FORWARD_THROTTLE, 0.0)
            else:
                throttle, steer = pd_follow(err, mem.last_err, dt_s)
                motors.arcade(throttle, steer)
                mem.last_err = err

            if task_sensors.pickup_target_reached(stack_name) or elapsed >= config.GRAB_FORWARD_MS:
                set_state(mem, states.GRAB_WAIT, "grab forward done -> wait")
            continue

        if mem.state == states.GRAB_WAIT:
            elapsed = ticks_diff(t_now, mem.state_t0)
            motors.arcade(0.0, 0.0)

            if not did_close:
                task_sensors.close_gripper()
                did_close = True

            if elapsed >= config.GRAB_WAIT_MS:
                if not task_sensors.reel_secured():
                    set_state(mem, states.STOP, "grab failed: reel not secured")
                    continue

                mem.grabbed_colour = task_sensors.identify_picked_reel(stack_name)
                if mem.grabbed_colour is not None:
                    task_sensors.show_detected_colour(mem.grabbed_colour)

                mem.acquire_ok = 0
                mem.last_err = 0.0 if err is None else err
                set_state(mem, states.GRAB_REVERSE, "grab wait done -> reverse to node")
            continue

        if mem.state == states.GRAB_REVERSE:
            elapsed = ticks_diff(t_now, mem.state_t0)
            if elapsed >= config.GRAB_REVERSE_TIMEOUT_MS:
                set_state(mem, states.STOP, "GRAB REVERSE TIMEOUT")
                continue

            throttle, steer = _reverse_follow(err, mem.last_err)
            motors.arcade(throttle, steer)

            if err is not None:
                mem.last_err = err

            node_seen = inter_cond or (sumw >= 3)
            if node_seen:
                mem.acquire_ok += 1
                if mem.acquire_ok >= config.GRAB_REVERSE_ALL_WHITE_N:
                    mem.last_event_t = t_now
                    mem.good_rearm = 0
                    set_state(mem, states.FOLLOW, "grab complete, back at pickup node")
            else:
                mem.acquire_ok = 0
            continue


def place_movement(mem, motors, sensors, states, task_sensors, colour_name, place_cfg):
    """
    Local place macro:
        enter bay from A/B/C/D
        move forward under task-sensor control
        release reel
        fixed-time reverse back to A/B/C/D
        perform explicit local exit action
        stop there and return to FOLLOW
    """
    t_last = ticks_ms()
    did_open = False

    enter_action = place_cfg.get("enter_action", "straight")
    exit_action = place_cfg.get("exit_action", "none")
    exit_spin_dir = place_cfg.get("exit_spin_dir", +1)

    while mem.state not in (states.FOLLOW, states.STOP):
        t_now, t_last = fixed_rate_tick(t_last, period_ms)
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(
            t_now, mem, corner_raw, inter_cond
        )
        err = middle_error_white_line(black)

        if mem.state == states.PLACE:
            ok = _do_local_enter_action(mem, motors, sensors, states, enter_action)
            if not ok:
                continue

            if mem.state == states.STOP:
                continue

            mem.last_err = 0.0 if err is None else err
            if mem.state != states.PLACE_FORWARD:
                set_state(mem, states.PLACE_FORWARD, "place entry complete -> forward")
            continue

        if mem.state == states.PLACE_FORWARD:
            elapsed = ticks_diff(t_now, mem.state_t0)

            if err is None:
                motors.arcade(PLACE_FORWARD_THROTTLE, 0.0)
            else:
                throttle, steer = pd_follow(err, mem.last_err, dt_s)
                motors.arcade(throttle, steer)
                mem.last_err = err

            if task_sensors.drop_target_reached(colour_name) or elapsed >= PLACE_FORWARD_TIMEOUT_MS:
                set_state(mem, states.PLACE_WAIT, "place forward done -> wait")
            continue

        if mem.state == states.PLACE_WAIT:
            elapsed = ticks_diff(t_now, mem.state_t0)
            motors.arcade(0.0, 0.0)

            if not did_open:
                task_sensors.open_gripper()
                did_open = True

            if elapsed >= PLACE_WAIT_MS:
                if not task_sensors.reel_released():
                    set_state(mem, states.STOP, "place failed: reel not released")
                    continue

                set_state(mem, states.PLACE_REVERSE, "place wait done -> fixed reverse out")
            continue

        if mem.state == states.PLACE_REVERSE:
            elapsed = ticks_diff(t_now, mem.state_t0)
            motors.arcade(-PLACE_REVERSE_THROTTLE_MAG, 0.0)

            if elapsed >= PLACE_REVERSE_MS:
                ok = _do_local_exit_action(
                    mem,
                    motors,
                    sensors,
                    states,
                    exit_action,
                    exit_spin_dir,
                )
                if not ok:
                    continue

                mem.last_event_t = t_now
                mem.good_rearm = 0

                if mem.state != states.FOLLOW:
                    set_state(mem, states.FOLLOW, "place complete, back at bay approach node")
                continue