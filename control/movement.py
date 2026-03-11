from utime import ticks_ms, ticks_diff, sleep_ms
import config
from control.ulilities import set_state, clamp
from control.pd import middle_error_white_line, pd_follow

period_ms = int(1000 / config.LOOP_HZ)
dt_s = period_ms / 1000.0

DEBUG = getattr(config, "DEBUG_MOTION", True)
DEBUG_EVERY = getattr(config, "DEBUG_EVERY", 5)  # print every N loops


def fixed_rate_tick(t_last, period_ms):
    """Returns (t_now, new_t_last)."""
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
        extra
    )


def compute_event(t_now, mem, corner_raw, inter_cond):

    # recent good line gate
    recent_good = ticks_diff(t_now, mem.t_last_good_line) <= config.RECENT_GOOD_LINE_MS

    # corner only counts if we were on the line recently
    corner_cond = recent_good and corner_raw and (not inter_cond)

    # event condition: either effective intersection or corner
    event_cond = inter_cond or corner_cond

    return inter_cond, corner_cond, event_cond, recent_good


def border_push_movement(mem, motors, sensors, states):

    t_last = ticks_ms()

    while mem.state != states.FOLLOW and mem.state != states.STOP:

        # Timing: enforce fixed loop rate and get current time
        t_now, t_last = fixed_rate_tick(t_last, period_ms)

        # Sense
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(t_now, mem, corner_raw, inter_cond)
        err = middle_error_white_line(black)

        # ---------------- START_BOX ----------------
        if mem.state == states.START_BOX:
            _dbg(
                mem, "START_BOX", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "see_white_count={}".format(mem.see_white_count)
            )

            motors.arcade(config.START_THROTTLE, 0.0)

            if sumw >= 1:
                mem.see_white_count += 1
                if mem.see_white_count >= config.START_SEE_WHITE_N:
                    mem.see_white_count = 0
                    mem.border_ok = 0
                    print("TRANSITION START_BOX -> BORDER_PUSH")
                    set_state(mem, states.BORDER_PUSH, "saw white at start, pushing border")
            else:
                mem.see_white_count = 0

            continue

        # ---------------- BORDER_PUSH ----------------
        if mem.state == states.BORDER_PUSH:
            elapsed = ticks_diff(t_now, mem.state_t0)

            if ticks_diff(t_now, mem.state_t0) >= config.BORDER_TIMEOUT_MS:
                print("BORDER_PUSH timeout")
                set_state(mem, states.STOP, "BORDER TIMEOUT")
                continue

            # Bias steering toward the side with more WHITE so we "home" to the exit.
            w = (-2 * white[0]) + (-1 * white[1]) + (1 * white[2]) + (2 * white[3])

            if w < 0:
                mem.last_search_dir = -1
            elif w > 0:
                mem.last_search_dir = +1

            steer = clamp(config.BORDER_BIAS_K * w, -config.BORDER_BIAS_MAX, +config.BORDER_BIAS_MAX)

            _dbg(
                mem, "BORDER_PUSH", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} w={} steer={} border_ok={}".format(elapsed, w, steer, mem.border_ok)
            )

            motors.arcade(config.BORDER_THROTTLE, steer)

            # Exit border when we are no longer on a fat patch (debounced)
            if sumw <= 2:
                mem.border_ok += 1
                if mem.border_ok >= config.BORDER_EXIT_N:
                    mem.border_ok = 0
                    mem.acquire_ok = 0
                    print("TRANSITION BORDER_PUSH -> ACQUIRE")
                    set_state(mem, states.ACQUIRE, "border cleared, acquiring clean line")
            else:
                mem.border_ok = 0

            continue

        # ---------------- ACQUIRE ----------------
        if mem.state == states.ACQUIRE:
            elapsed = ticks_diff(t_now, mem.state_t0)

            _dbg(
                mem, "ACQUIRE", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} acquire_ok={} last_search_dir={}".format(
                    elapsed, mem.acquire_ok, mem.last_search_dir
                )
            )

            if ticks_diff(t_now, mem.state_t0) >= config.ACQUIRE_TIMEOUT_MS:
                print("ACQUIRE timeout")
                set_state(mem, states.STOP, "ACQUIRE TIMEOUT")
                continue

            if sumw >= 3:
                mem.border_ok = 0
                print("ACQUIRE saw wide white -> BORDER_PUSH")
                set_state(mem, states.BORDER_PUSH, "wide white returned during acquire")
                continue

            if sumw == 0:
                motors.arcade(0.0, mem.last_search_dir * config.ACQUIRE_SEARCH_STEER)
                continue

            # steer toward white using all sensors
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
                    print("TRANSITION ACQUIRE -> GATE_CLEAR")
                    set_state(mem, states.GATE_CLEAR, "acquired good line, gate clear")
            else:
                mem.acquire_ok = 0

            continue

        # ---------------- GATE_CLEAR ----------------
        if mem.state == states.GATE_CLEAR:
            elapsed = ticks_diff(t_now, mem.state_t0)

            _dbg(
                mem, "GATE_CLEAR", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={}".format(elapsed)
            )

            motors.arcade(config.BORDER_THROTTLE, 0.0)

            if ticks_diff(t_now, mem.state_t0) >= config.GATE_CLEAR_MS:
                mem.events_armed = True
                print("TRANSITION GATE_CLEAR -> FOLLOW")
                set_state(mem, states.FOLLOW, "gate cleared, arm wait")

            continue


def turning_movement(mem, motors, sensors, states, advance_step=True):
    t_last = ticks_ms()

    while mem.state not in (states.FOLLOW, states.STOP, states.GRAB):

        # Timing: enforce fixed loop rate and get current time
        t_now, t_last = fixed_rate_tick(t_last, period_ms)

        # Sense
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(t_now, mem, corner_raw, inter_cond)
        err = middle_error_white_line(black)

        # ---------------- TURN_APPROACH ----------------
        if mem.state == states.TURN_APPROACH:
            elapsed = ticks_diff(t_now, mem.state_t0)

            _dbg(
                mem, "TURN_APPROACH", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} dir_turn={}".format(elapsed, mem.dir_turn)
            )

            motors.arcade(config.STRAIGHT_THROTTLE, 0.0)

            if ticks_diff(t_now, mem.state_t0) >= config.TURN_APPROACH_MS:
                print("TRANSITION TURN_APPROACH -> TURN_SPIN")
                set_state(mem, states.TURN_SPIN, "approach done -> spin")

            continue

        # ---------------- TURN_SPIN ----------------
        if mem.state == states.TURN_SPIN:
            elapsed = ticks_diff(t_now, mem.state_t0)

            if elapsed >= config.TURN_TIMEOUT_MS:
                print("TURN_SPIN timeout step=", mem.step)
                set_state(mem, states.STOP, "TURN TIMEOUT step={}".format(mem.step))
                continue

            _dbg(
                mem, "TURN_SPIN", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} dir_turn={}".format(elapsed, mem.dir_turn)
            )

            motors.arcade(config.TURN_THROTTLE, mem.dir_turn * config.TURN_STEER)

            if elapsed < config.TURN_MIN_MS:
                continue

            if mem.dir_turn > 0:
                sighted = (white[2] == 1) or (white[3] == 1)
            else:
                sighted = (white[0] == 1) or (white[1] == 1)

            if sighted:
                print("TURN_SPIN sighted new branch")
                mem.acquire_ok = 0
                set_state(mem, states.TURN_ALIGN, "sighted new branch, aligning")

            continue

        # ---------------- TURN_ALIGN ----------------
        if mem.state == states.TURN_ALIGN:
            elapsed = ticks_diff(t_now, mem.state_t0)

            _dbg(
                mem, "TURN_ALIGN", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} acquire_ok={} dir_turn={}".format(elapsed, mem.acquire_ok, mem.dir_turn)
            )

            if ticks_diff(t_now, mem.state_t0) >= config.ALIGN_TIMEOUT_MS:
                print("TURN_ALIGN timeout step=", mem.step)
                set_state(mem, states.STOP, "ALIGN TIMEOUT step={}".format(mem.step))
                continue

            if err is None:
                motors.arcade(config.ALIGN_THROTTLE, mem.dir_turn * 0.35)
            else:
                motors.arcade(config.ALIGN_THROTTLE, clamp(0.20 * err, -0.35, +0.35))

            if good_line and (not inter_cond) and (sumw <= 2):
                mem.acquire_ok += 1
                if mem.acquire_ok >= config.REACQUIRE_N:
                    print("DONE turn. step=", mem.step, "dir=", ("R" if mem.dir_turn > 0 else "L"))
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

        # Timing: enforce fixed loop rate and get current time
        t_now, t_last = fixed_rate_tick(t_last, period_ms)

        # Sense
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(t_now, mem, corner_raw, inter_cond)
        err = middle_error_white_line(black)

        if mem.state == states.DO_STRAIGHT:
            elapsed = ticks_diff(t_now, mem.state_t0)

            steer = 0.0
            if err is not None:
                steer = clamp(
                    config.STRAIGHT_KP * err,
                    -config.STRAIGHT_MAX_STEER,
                    +config.STRAIGHT_MAX_STEER
                )

            _dbg(
                mem, "DO_STRAIGHT", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} steer={} straight_out={}".format(elapsed, steer, mem.straight_out)
            )

            motors.arcade(config.STRAIGHT_THROTTLE, steer)

            if (not inter_cond) and (sumw <= 2):
                mem.straight_out += 1
            else:
                mem.straight_out = 0

            if elapsed >= config.STRAIGHT_MIN_MS and mem.straight_out >= config.INTER_EXIT_N and good_line:
                print("DONE straight. step=", mem.step)
                mem.step += 1
                mem.last_event_t = t_now
                mem.good_rearm = 0
                set_state(mem, states.FOLLOW, "straight complete")
                continue

            if elapsed >= config.STRAIGHT_TIMEOUT_MS:
                print("DO_STRAIGHT timeout")
                mem.last_event_t = t_now
                mem.good_rearm = 0
                set_state(mem, states.STOP, "straight timeout -> stop")
                continue


def spin180_movement(mem, motors, sensors, states):
    """
    Blocking helper for a 180-degree turn.

    Expected states:
        states.SPIN180_SPIN
        states.SPIN180_ALIGN

    Exits when mem.state becomes:
        states.FOLLOW
        states.STOP
    """
    t_last = ticks_ms()

    while mem.state not in (states.FOLLOW, states.STOP):

        # Timing: enforce fixed loop rate and get current time
        t_now, t_last = fixed_rate_tick(t_last, period_ms)

        # Sense
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(t_now, mem, corner_raw, inter_cond)
        err = middle_error_white_line(black)

        # ---------------- SPIN180_SPIN ----------------
        if mem.state == states.SPIN180_SPIN:
            elapsed = ticks_diff(t_now, mem.state_t0)

            _dbg(
                mem, "SPIN180_SPIN", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} dir_turn={}".format(elapsed, mem.dir_turn)
            )

            if elapsed >= config.SPIN180_TIMEOUT_MS:
                print("SPIN180_SPIN timeout")
                set_state(mem, states.STOP, "SPIN180 TIMEOUT")
                continue

            motors.arcade(config.SPIN180_THROTTLE, mem.dir_turn * config.SPIN180_STEER)

            if elapsed < config.SPIN180_MIN_MS:
                continue

            # after minimum spin, switch to align once line is sighted again
            sighted = (sumw >= 1)
            if sighted:
                print("SPIN180_SPIN sighted line")
                mem.acquire_ok = 0
                set_state(mem, states.SPIN180_ALIGN, "180 spin sighted line -> align")

            continue

        # ---------------- SPIN180_ALIGN ----------------
        if mem.state == states.SPIN180_ALIGN:
            elapsed = ticks_diff(t_now, mem.state_t0)

            _dbg(
                mem, "SPIN180_ALIGN", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} acquire_ok={}".format(elapsed, mem.acquire_ok)
            )

            if ticks_diff(t_now, mem.state_t0) >= config.SPIN180_ALIGN_TIMEOUT_MS:
                print("SPIN180_ALIGN timeout")
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
                    print("SPIN180 complete")
                    mem.last_event_t = t_now
                    mem.good_rearm = 0
                    set_state(mem, states.FOLLOW, "180 spin complete")
            else:
                mem.acquire_ok = 0

            continue


def grab_movement(mem, motors, sensors, states):
    t_last = ticks_ms()

    while mem.state not in (states.FOLLOW, states.STOP):

        # Timing: enforce fixed loop rate and get current time
        t_now, t_last = fixed_rate_tick(t_last, period_ms)

        # Sense
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(t_now, mem, corner_raw, inter_cond)
        err = middle_error_white_line(black)

        # ---------------- GRAB ----------------
        if mem.state == states.GRAB:
            _dbg(
                mem, "GRAB", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err
            )

            print("TRANSITION GRAB -> TURN_SPIN")
            set_state(mem, states.TURN_SPIN, "grab -> first turn")
            turning_movement(mem, motors, sensors, states, advance_step=False)

            if mem.state == states.STOP:
                continue

            mem.last_err = 0.0 if err is None else err
            print("TRANSITION TURN -> GRAB_FORWARD")
            set_state(mem, states.GRAB_FORWARD, "grab turn complete -> forward")
            continue

        # ---------------- GRAB_FORWARD ----------------
        if mem.state == states.GRAB_FORWARD:
            elapsed = ticks_diff(t_now, mem.state_t0)

            _dbg(
                mem, "GRAB_FORWARD", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} last_err={}".format(elapsed, mem.last_err)
            )

            if err is None:
                motors.arcade(config.GRAB_FORWARD_THROTTLE, 0.0)
            else:
                throttle, steer = pd_follow(err, mem.last_err, dt_s)
                motors.arcade(throttle, steer)
                mem.last_err = err

            if ticks_diff(t_now, mem.state_t0) >= config.GRAB_FORWARD_MS:
                print("GRAB_FORWARD complete")
                set_state(mem, states.GRAB_WAIT, "grab forward done -> wait")
            continue

        # ---------------- GRAB_WAIT ----------------
        if mem.state == states.GRAB_WAIT:
            elapsed = ticks_diff(t_now, mem.state_t0)

            _dbg(
                mem, "GRAB_WAIT", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={}".format(elapsed)
            )

            motors.arcade(0.0, 0.0)

            if ticks_diff(t_now, mem.state_t0) >= config.GRAB_WAIT_MS:
                mem.acquire_ok = 0
                mem.last_err = 0.0 if err is None else err
                print("GRAB_WAIT complete")
                set_state(mem, states.GRAB_REVERSE, "grab wait done -> reverse")
            continue

        # ---------------- GRAB_REVERSE ----------------
        if mem.state == states.GRAB_REVERSE:
            elapsed = ticks_diff(t_now, mem.state_t0)

            _dbg(
                mem, "GRAB_REVERSE", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err,
                "elapsed={} acquire_ok={} last_err={}".format(elapsed, mem.acquire_ok, mem.last_err)
            )

            if ticks_diff(t_now, mem.state_t0) >= config.GRAB_REVERSE_TIMEOUT_MS:
                print("GRAB_REVERSE timeout")
                set_state(mem, states.STOP, "GRAB REVERSE TIMEOUT")
                continue

            if err is None:
                motors.arcade(config.GRAB_REVERSE_THROTTLE, 0.0)
            else:
                throttle, steer = pd_follow(err, mem.last_err, dt_s)
                motors.arcade(-throttle, -steer)
                mem.last_err = err

            if sumw == 4:
                mem.acquire_ok += 1
                if mem.acquire_ok >= config.GRAB_REVERSE_ALL_WHITE_N:
                    print("GRAB_REVERSE complete -> GRAB_TURN_BACK")
                    set_state(mem, states.GRAB_TURN_BACK, "grab reverse done -> turn back")
            else:
                mem.acquire_ok = 0

            continue

        # ---------------- GRAB_TURN_BACK ----------------
        if mem.state == states.GRAB_TURN_BACK:
            _dbg(
                mem, "GRAB_TURN_BACK", t_now, black, white, sumw, good_line,
                inter_cond, corner_raw, corner_cond, event_cond, recent_good, err
            )

            print("TRANSITION GRAB_TURN_BACK -> TURN_APPROACH")
            set_state(mem, states.TURN_APPROACH, "grab -> second turn")
            turning_movement(mem, motors, sensors, states, advance_step=False)

            if mem.state == states.STOP:
                continue

            print("GRAB complete -> FOLLOW")
            set_state(mem, states.FOLLOW, "grab complete")
            continue