from utime import ticks_ms, ticks_diff
import config
from control.ulilities import set_state, clamp
from control.pd import middle_error_white_line


def border_push_movement(mem, motors, sensors, states):
    while mem.state != states.FOLLOW and mem.state != states.STOP:
        t_now = ticks_ms()
        black, white, sumw, good_line = sensors.sense()

        # ---------------- START_BOX ----------------
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

        # ---------------- BORDER_PUSH ----------------
        if mem.state == states.BORDER_PUSH:
            if ticks_diff(t_now, mem.state_t0) >= config.BORDER_TIMEOUT_MS:
                set_state(mem, states.STOP, "BORDER TIMEOUT")
                continue

            # Bias steering toward the side with more WHITE so we "home" to the exit.
            w = (-2 * white[0]) + (-1 * white[1]) + (1 * white[2]) + (2 * white[3])

            if w < 0:
                mem.last_search_dir = -1
            elif w > 0:
                mem.last_search_dir = +1

            steer = clamp(config.BORDER_BIAS_K * w, -config.BORDER_BIAS_MAX, +config.BORDER_BIAS_MAX)
            motors.arcade(config.BORDER_THROTTLE, steer)

            # Exit border when we are no longer on a fat patch (debounced)
            if sumw <= 2:
                mem.border_ok += 1
                if mem.border_ok >= config.BORDER_EXIT_N:
                    mem.border_ok = 0
                    mem.acquire_ok = 0
                    set_state(mem, states.ACQUIRE, "border cleared, acquiring clean line")
            else:
                mem.border_ok = 0

            continue

        # ---------------- ACQUIRE ----------------
        if mem.state == states.ACQUIRE:
            if ticks_diff(t_now, mem.state_t0) >= config.ACQUIRE_TIMEOUT_MS:
                set_state(mem, states.STOP, "ACQUIRE TIMEOUT")
                continue

            if sumw >= 3:
                mem.border_ok = 0
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
                    set_state(mem, states.GATE_CLEAR, "acquired good line, gate clear")
            else:
                mem.acquire_ok = 0

            continue

        # ---------------- GATE_CLEAR ----------------
        if mem.state == states.GATE_CLEAR:
            motors.arcade(config.BORDER_THROTTLE, 0.0)

            if ticks_diff(t_now, mem.state_t0) >= config.GATE_CLEAR_MS:
                mem.events_armed = True
                set_state(mem, states.FOLLOW, "gate cleared, arm wait")

            continue


def turning_movement(mem, motors, sensors, states):
    while mem.state not in (states.FOLLOW, states.STOP):
        t_now = ticks_ms()
        black, white, sumw, good_line = sensors.sense()

        inter_cond = sensors.is_intersection(white)

        # ---------------- TURN_APPROACH ----------------
        if mem.state == states.TURN_APPROACH:
            motors.arcade(config.STRAIGHT_THROTTLE, 0.0)

            if ticks_diff(t_now, mem.state_t0) >= config.TURN_APPROACH_MS:
                set_state(mem, states.TURN_SPIN, "approach done -> spin")

            continue

        # ---------------- TURN_SPIN ----------------
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
                set_state(mem, states.TURN_ALIGN, "sighted new branch, aligning")

            continue

        # ---------------- TURN_ALIGN ----------------
        if mem.state == states.TURN_ALIGN:
            if ticks_diff(t_now, mem.state_t0) >= config.ALIGN_TIMEOUT_MS:
                set_state(mem, states.STOP, "ALIGN TIMEOUT step={}".format(mem.step))
                continue

            err = middle_error_white_line(black)

            if err is None:
                motors.arcade(config.ALIGN_THROTTLE, mem.dir_turn * 0.35)
            else:
                motors.arcade(config.ALIGN_THROTTLE, clamp(0.20 * err, -0.35, +0.35))

            if good_line and (not inter_cond) and (sumw <= 2):
                mem.acquire_ok += 1
                if mem.acquire_ok >= config.REACQUIRE_N:
                    print("DONE turn. step=", mem.step, "dir=", ("R" if mem.dir_turn > 0 else "L"))
                    mem.step += 1
                    mem.last_event_t = t_now
                    mem.good_rearm = 0
                    set_state(mem, states.FOLLOW, "turn complete")
            else:
                mem.acquire_ok = 0

            continue

    return mem.state == states.STOP

def straight_movement(mem, motors, sensors, states, mission):
    while mem.state not in (states.FOLLOW, states.FINAL_FOLLOW, states.STOP):
        t_now = ticks_ms()
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()
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

                if mem.finish_mode and mem.step >= len(mission):
                    set_state(mem, states.FINAL_FOLLOW, "final straight complete -> final follow")
                else:
                    set_state(mem, states.FOLLOW, "straight complete")
                continue

            if elapsed >= config.STRAIGHT_TIMEOUT_MS:
                mem.last_event_t = t_now
                mem.good_rearm = 0
                set_state(mem, states.FOLLOW, "straight timeout -> follow")
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

    while mem.state not in (states.FOLLOW, states.STOP):
        t_now = ticks_ms()
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()
        err = middle_error_white_line(black)

        # ---------------- SPIN180_SPIN ----------------
        if mem.state == states.SPIN180_SPIN:
            elapsed = ticks_diff(t_now, mem.state_t0)

            if elapsed >= config.SPIN180_TIMEOUT_MS:
                set_state(mem, states.STOP, "SPIN180 TIMEOUT")
                continue

            motors.arcade(config.SPIN180_THROTTLE, mem.dir_turn * config.SPIN180_STEER)

            if elapsed < config.SPIN180_MIN_MS:
                continue

            # after minimum spin, switch to align once line is sighted again
            sighted = (sumw >= 1)
            if sighted:
                mem.acquire_ok = 0
                set_state(mem, states.SPIN180_ALIGN, "180 spin sighted line -> align")

            continue

        # ---------------- SPIN180_ALIGN ----------------
        if mem.state == states.SPIN180_ALIGN:
            if ticks_diff(t_now, mem.state_t0) >= config.SPIN180_ALIGN_TIMEOUT_MS:
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
                    mem.last_event_t = t_now
                    mem.good_rearm = 0
                    set_state(mem, states.FOLLOW, "180 spin complete")
            else:
                mem.acquire_ok = 0

            continue

    return mem.state == states.STOP

def grab_movement(mem, motors, sensors, states):
    None