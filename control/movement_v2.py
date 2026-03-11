from utime import ticks_ms, ticks_diff, sleep_ms

import config
from control.ulilities import set_state, clamp
from control.pd import middle_error_white_line, pd_follow

period_ms = int(1000 / config.LOOP_HZ)
dt_s = period_ms / 1000.0

DEBUG = getattr(config, "DEBUG_MOTION", True)
DEBUG_EVERY = getattr(config, "DEBUG_EVERY", 5)


def fixed_rate_tick(t_last, period_ms):
    t_now = ticks_ms()
    dt = ticks_diff(t_now, t_last)
    if dt < period_ms:
        sleep_ms(period_ms - dt)
        t_now = ticks_ms()
    return t_now, t_now


def _bits(x):
    return "".join("1" if v else "0" for v in x)


def _dbg(mem, label, t_now, black, white, sumw, good_line,
         inter_cond, corner_raw, corner_cond, event_cond, recent_good,
         err, extra=""):
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

        inter_cond, corner_cond, event_cond, recent_good = compute_event(t_now, mem, corner_raw, inter_cond)
        err = middle_error_white_line(black)

        if mem.state == states.START_BOX:
            _dbg(
                mem, "START_BOX", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "see_white_count={}".format(mem.see_white_count),
            )

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

            steer = clamp(config.BORDER_BIAS_K * w, -config.BORDER_BIAS_MAX, +config.BORDER_BIAS_MAX)
            _dbg(
                mem, "BORDER_PUSH", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} w={} steer={} border_ok={}".format(elapsed, w, steer, mem.border_ok),
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
            _dbg(
                mem, "ACQUIRE", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} acquire_ok={} last_search_dir={}".format(
                    elapsed, mem.acquire_ok, mem.last_search_dir
                ),
            )

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
            elapsed = ticks_diff(t_now, mem.state_t0)
            _dbg(
                mem, "GATE_CLEAR", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={}".format(elapsed),
            )

            motors.arcade(config.BORDER_THROTTLE, 0.0)
            if elapsed >= config.GATE_CLEAR_MS:
                mem.events_armed = True
                set_state(mem, states.FOLLOW, "gate cleared")
            continue


def turning_movement(mem, motors, sensors, states, advance_step=True):
    t_last = ticks_ms()

    while mem.state not in (states.FOLLOW, states.STOP, states.GRAB, states.PLACE):
        t_now, t_last = fixed_rate_tick(t_last, period_ms)
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(t_now, mem, corner_raw, inter_cond)
        err = middle_error_white_line(black)

        if mem.state == states.TURN_APPROACH:
            elapsed = ticks_diff(t_now, mem.state_t0)
            _dbg(
                mem, "TURN_APPROACH", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} dir_turn={}".format(elapsed, mem.dir_turn),
            )

            motors.arcade(config.STRAIGHT_THROTTLE, 0.0)
            if elapsed >= config.TURN_APPROACH_MS:
                set_state(mem, states.TURN_SPIN, "approach done -> spin")
            continue

        if mem.state == states.TURN_SPIN:
            elapsed = ticks_diff(t_now, mem.state_t0)
            if elapsed >= config.TURN_TIMEOUT_MS:
                set_state(mem, states.STOP, "TURN TIMEOUT step={}".format(mem.step))
                continue

            _dbg(
                mem, "TURN_SPIN", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} dir_turn={}".format(elapsed, mem.dir_turn),
            )

            motors.arcade(config.TURN_THROTTLE, mem.dir_turn * config.TURN_STEER)
            if elapsed < config.TURN_MIN_MS:
                continue

            if mem.dir_turn > 0:
                sighted = (white[2] == 1) or (white[3] == 1)
            else:
                sighted = (white[0] == 1) or (white[1] == 1)

            if sighted:
                mem.acquire_ok = 0
                set_state(mem, states.TURN_ALIGN, "sighted new branch, aligning")
            continue

        if mem.state == states.TURN_ALIGN:
            elapsed = ticks_diff(t_now, mem.state_t0)
            _dbg(
                mem, "TURN_ALIGN", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} acquire_ok={} dir_turn={}".format(elapsed, mem.acquire_ok, mem.dir_turn),
            )

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


def straight_movement(mem, motors, sensors, states, mission):
    t_last = ticks_ms()

    while mem.state not in (states.FOLLOW, states.STOP):
        t_now, t_last = fixed_rate_tick(t_last, period_ms)
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(t_now, mem, corner_raw, inter_cond)
        err = middle_error_white_line(black)

        if mem.state == states.DO_STRAIGHT:
            elapsed = ticks_diff(t_now, mem.state_t0)
            steer = 0.0
            if err is not None:
                steer = clamp(config.STRAIGHT_KP * err, -config.STRAIGHT_MAX_STEER, +config.STRAIGHT_MAX_STEER)

            _dbg(
                mem, "DO_STRAIGHT", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} steer={} straight_out={}".format(elapsed, steer, mem.straight_out),
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
                set_state(mem, states.STOP, "straight timeout -> stop")
                continue


def spin180_movement(mem, motors, sensors, states, advance_step=True):
    t_last = ticks_ms()

    while mem.state not in (states.FOLLOW, states.STOP):
        t_now, t_last = fixed_rate_tick(t_last, period_ms)
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(t_now, mem, corner_raw, inter_cond)
        err = middle_error_white_line(black)

        if mem.state == states.SPIN180_SPIN:
            elapsed = ticks_diff(t_now, mem.state_t0)
            _dbg(
                mem, "SPIN180_SPIN", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} dir_turn={}".format(elapsed, mem.dir_turn),
            )

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
            _dbg(
                mem, "SPIN180_ALIGN", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} acquire_ok={}".format(elapsed, mem.acquire_ok),
            )

            if elapsed >= config.SPIN180_ALIGN_TIMEOUT_MS:
                set_state(mem, states.STOP, "SPIN180 ALIGN TIMEOUT")
                continue

            if err is None:
                motors.arcade(config.SPIN180_ALIGN_THROTTLE, mem.dir_turn * 0.35)
            else:
                motors.arcade(
                    config.SPIN180_ALIGN_THROTTLE,
                    clamp(config.SPIN180_ALIGN_KP * err, -config.SPIN180_ALIGN_MAX_STEER, +config.SPIN180_ALIGN_MAX_STEER),
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


def grab_movement(mem, motors, sensors, states, task_sensors, stack_name):
    t_last = ticks_ms()
    turn_in = mem.dir_turn
    turn_out = -turn_in
    did_close = False

    while mem.state not in (states.FOLLOW, states.STOP):
        t_now, t_last = fixed_rate_tick(t_last, period_ms)
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(t_now, mem, corner_raw, inter_cond)
        err = middle_error_white_line(black)

        if mem.state == states.GRAB:
            _dbg(
                mem, "GRAB", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "turn_in={} turn_out={}".format(turn_in, turn_out),
            )

            mem.dir_turn = turn_in
            set_state(mem, states.TURN_SPIN, "grab -> enter branch")
            turning_movement(mem, motors, sensors, states, advance_step=False)
            if mem.state == states.STOP:
                continue

            mem.last_err = 0.0 if err is None else err
            set_state(mem, states.GRAB_FORWARD, "grab entry turn complete -> forward")
            continue

        if mem.state == states.GRAB_FORWARD:
            elapsed = ticks_diff(t_now, mem.state_t0)
            _dbg(
                mem, "GRAB_FORWARD", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} last_err={}".format(elapsed, mem.last_err),
            )

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
            _dbg(
                mem, "GRAB_WAIT", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} did_close={}".format(elapsed, did_close),
            )

            motors.arcade(0.0, 0.0)
            if not did_close:
                task_sensors.close_gripper()
                did_close = True

            if elapsed >= config.GRAB_WAIT_MS:
                mem.grabbed_colour = task_sensors.identify_picked_reel(stack_name)
                if mem.grabbed_colour is not None:
                    task_sensors.show_detected_colour(mem.grabbed_colour)
                mem.acquire_ok = 0
                mem.last_err = 0.0 if err is None else err
                set_state(mem, states.GRAB_REVERSE, "grab wait done -> reverse to junction")
            continue

        if mem.state == states.GRAB_REVERSE:
            elapsed = ticks_diff(t_now, mem.state_t0)
            _dbg(
                mem, "GRAB_REVERSE", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} acquire_ok={} last_err={}".format(elapsed, mem.acquire_ok, mem.last_err),
            )

            if elapsed >= config.GRAB_REVERSE_TIMEOUT_MS:
                set_state(mem, states.STOP, "GRAB REVERSE TIMEOUT")
                continue

            throttle, steer = _reverse_follow(err, mem.last_err)
            motors.arcade(throttle, steer)
            if err is not None:
                mem.last_err = err

            if sumw == 4:
                mem.acquire_ok += 1
                if mem.acquire_ok >= config.GRAB_REVERSE_ALL_WHITE_N:
                    set_state(mem, states.GRAB_TURN_BACK, "junction reacquired -> exit branch")
            else:
                mem.acquire_ok = 0
            continue

        if mem.state == states.GRAB_TURN_BACK:
            _dbg(
                mem, "GRAB_TURN_BACK", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "turn_out={}".format(turn_out),
            )

            mem.dir_turn = turn_out
            set_state(mem, states.TURN_SPIN, "grab -> exit branch")
            turning_movement(mem, motors, sensors, states, advance_step=False)
            if mem.state == states.STOP:
                continue

            mem.last_event_t = ticks_ms()
            mem.good_rearm = 0
            set_state(mem, states.FOLLOW, "grab complete")
            continue


def place_movement(mem, motors, sensors, states, task_sensors, colour_name):
    t_last = ticks_ms()
    turn_in = mem.dir_turn
    turn_out = -turn_in
    did_open = False

    while mem.state not in (states.FOLLOW, states.STOP):
        t_now, t_last = fixed_rate_tick(t_last, period_ms)
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(t_now, mem, corner_raw, inter_cond)
        err = middle_error_white_line(black)

        if mem.state == states.PLACE:
            _dbg(
                mem, "PLACE", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "turn_in={} turn_out={} colour={}".format(turn_in, turn_out, colour_name),
            )

            mem.dir_turn = turn_in
            set_state(mem, states.TURN_SPIN, "place -> enter bay")
            turning_movement(mem, motors, sensors, states, advance_step=False)
            if mem.state == states.STOP:
                continue

            mem.last_err = 0.0 if err is None else err
            set_state(mem, states.PLACE_FORWARD, "place entry turn complete -> forward")
            continue

        if mem.state == states.PLACE_FORWARD:
            elapsed = ticks_diff(t_now, mem.state_t0)
            _dbg(
                mem, "PLACE_FORWARD", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} colour={}".format(elapsed, colour_name),
            )

            if err is None:
                motors.arcade(config.GRAB_FORWARD_THROTTLE, 0.0)
            else:
                throttle, steer = pd_follow(err, mem.last_err, dt_s)
                motors.arcade(throttle, steer)
                mem.last_err = err

            if task_sensors.drop_target_reached(colour_name) or elapsed >= config.GRAB_FORWARD_MS:
                set_state(mem, states.PLACE_WAIT, "place forward done -> wait")
            continue

        if mem.state == states.PLACE_WAIT:
            elapsed = ticks_diff(t_now, mem.state_t0)
            _dbg(
                mem, "PLACE_WAIT", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} did_open={}".format(elapsed, did_open),
            )

            motors.arcade(0.0, 0.0)
            if not did_open:
                task_sensors.open_gripper()
                did_open = True

            if elapsed >= config.GRAB_WAIT_MS:
                mem.acquire_ok = 0
                mem.last_err = 0.0 if err is None else err
                set_state(mem, states.PLACE_REVERSE, "place wait done -> reverse to junction")
            continue

        if mem.state == states.PLACE_REVERSE:
            elapsed = ticks_diff(t_now, mem.state_t0)
            _dbg(
                mem, "PLACE_REVERSE", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} acquire_ok={} last_err={}".format(elapsed, mem.acquire_ok, mem.last_err),
            )

            if elapsed >= config.GRAB_REVERSE_TIMEOUT_MS:
                set_state(mem, states.STOP, "PLACE REVERSE TIMEOUT")
                continue

            throttle, steer = _reverse_follow(err, mem.last_err)
            motors.arcade(throttle, steer)
            if err is not None:
                mem.last_err = err

            if sumw == 4:
                mem.acquire_ok += 1
                if mem.acquire_ok >= config.GRAB_REVERSE_ALL_WHITE_N:
                    set_state(mem, states.PLACE_TURN_BACK, "junction reacquired -> exit bay")
            else:
                mem.acquire_ok = 0
            continue

        if mem.state == states.PLACE_TURN_BACK:
            _dbg(
                mem, "PLACE_TURN_BACK", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "turn_out={}".format(turn_out),
            )

            mem.dir_turn = turn_out
            set_state(mem, states.TURN_SPIN, "place -> exit bay")
            turning_movement(mem, motors, sensors, states, advance_step=False)
            if mem.state == states.STOP:
                continue

            mem.last_event_t = ticks_ms()
            mem.good_rearm = 0
            set_state(mem, states.FOLLOW, "place complete")
            continue