from utime import ticks_ms, ticks_diff, sleep_ms

import config
from hw.line import LineSensors
from hw.motors import DCMotor, MotorPair
from control.ulilities import b4, set_state
from control.runtime_variables_v3 import Mem, State
from control.movement_v3 import (
    border_push_movement,
    turning_movement,
    straight_movement,
    spin180_movement,
    grab_movement,
    place_movement,
)
from control.pd import middle_error_white_line, pd_follow
from state_machine_3 import navigation
from hw.task_sensors import TaskSensors


DEBUG = True
DEBUG_LOOP_MS = 200


def dbg(*args):
    if DEBUG:
        print("[DBG]", *args)


def fixed_rate_tick(t_last, period_ms):
    t_now = ticks_ms()
    dt = ticks_diff(t_now, t_last)
    if dt < period_ms:
        sleep_ms(period_ms - dt)
        t_now = ticks_ms()
    return t_now, t_now


def compute_event(t_now, mem, corner_raw, inter_cond):
    recent_good = ticks_diff(t_now, mem.t_last_good_line) <= config.RECENT_GOOD_LINE_MS
    corner_cond = recent_good and corner_raw and (not inter_cond)
    event_cond = inter_cond or corner_cond
    return inter_cond, corner_cond, event_cond, recent_good


def _reset_after_macro(mem):
    mem.last_event_t = ticks_ms()
    mem.good_rearm = 0
    mem.event_in_count = 0
    mem.last_err = 0.0


def _build_initial_mission(nav):
    mission = nav.build_initial_scan_mission()
    dbg("INITIAL SCAN CAMPAIGN BUILT")
    return mission


def _maybe_wait_for_start(task_sensors, motors):
    while not task_sensors.start_pressed():
        motors.arcade(0.0, 0.0)
        sleep_ms(20)


def _dispatch_action(mem, states, step, inter_cond):
    act = step["move"]

    if act == "straight":
        mem.straight_out = 0
        set_state(mem, states.DO_STRAIGHT, "event->straight")
        return True

    if act == "left":
        mem.dir_turn = -1
        if inter_cond:
            set_state(mem, states.TURN_APPROACH, "event->left (approach)")
        else:
            set_state(mem, states.TURN_SPIN, "event->left (corner)")
        return True

    if act == "right":
        mem.dir_turn = +1
        if inter_cond:
            set_state(mem, states.TURN_APPROACH, "event->right (approach)")
        else:
            set_state(mem, states.TURN_SPIN, "event->right (corner)")
        return True

    if act == "180":
        mem.dir_turn = step.get("spin_dir", +1)
        set_state(mem, states.SPIN180_SPIN, "event->180")
        return True

    return False


def main():
    sensors = LineSensors(config.LINE_PINS, invert=config.LINE_INVERT)

    left = DCMotor(
        config.MOTOR_L_DIR,
        config.MOTOR_L_PWM,
        pwm_freq_hz=config.MOTOR_PWM_FREQ_HZ,
        invert=config.MOTOR_L_INVERT,
    )
    right = DCMotor(
        config.MOTOR_R_DIR,
        config.MOTOR_R_PWM,
        pwm_freq_hz=config.MOTOR_PWM_FREQ_HZ,
        invert=config.MOTOR_R_INVERT,
    )
    motors = MotorPair(left, right)

    period_ms = int(1000 / config.LOOP_HZ)
    dt_s = period_ms / 1000.0

    mem = Mem()
    states = State()
    nav = navigation(expected_total_reels=4)
    task_sensors = TaskSensors()

    t_last = ticks_ms()
    last_dbg_t = t_last

    _maybe_wait_for_start(task_sensors, motors)

    mission = _build_initial_mission(nav)
    border_push_movement(mem, motors, sensors, states)

    while True:
        t_now, t_last = fixed_rate_tick(t_last, period_ms)

        black, white, sumw, good_line, inter_raw, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(
            t_now, mem, corner_raw, inter_raw
        )
        err = middle_error_white_line(black)

        if ticks_diff(t_now, last_dbg_t) >= DEBUG_LOOP_MS:
            dbg(
                "LOOP",
                "state=", mem.state,
                "step=", mem.step,
                "mode=", nav.mode,
                "stack=", nav.active_stack,
                "slot=", nav.active_slot,
                "sumw=", sumw,
                "black=", b4(black),
                "white=", b4(white),
                "good=", good_line,
                "inter=", inter_cond,
                "corner=", corner_cond,
                "event=", event_cond,
                "err=", err,
                "armed=", mem.events_armed,
                "rearm=", mem.good_rearm,
                "delivered=", nav.total_delivered(),
                "remaining=", nav.reels_remaining_to_find(),
            )
            last_dbg_t = t_now

        if task_sensors.stop_pressed():
            set_state(mem, states.STOP, "external stop pressed")

        if mem.state == states.STOP:
            motors.arcade(0.0, 0.0)
            break

        if mem.state == states.FOLLOW:
            step = nav.get_step(mission, mem.step)

            if step is None:
                dbg("MISSION EXHAUSTED", "mode=", nav.mode)

                if nav.mode in ("scan", "resume_scan"):
                    if nav.all_resolved():
                        mission = nav.build_return_home_mission()
                        mem.step = 0
                        _reset_after_macro(mem)
                    else:
                        set_state(
                            mem,
                            states.STOP,
                            "scan campaign exhausted before all expected reels were delivered",
                        )
                    continue

                if nav.mode == "to_place_approach":
                    set_state(
                        mem,
                        states.PLACE,
                        "arrived at bay approach {}".format(nav.place_approach_node),
                    )
                    continue

                if nav.mode == "return_home":
                    set_state(mem, states.STOP, "all reels delivered; back at node 1")
                    continue

                set_state(mem, states.STOP, "unknown nav mode {}".format(nav.mode))
                continue

            if err is not None:
                if err > 0:
                    mem.last_search_dir = +1
                elif err < 0:
                    mem.last_search_dir = -1

            if recent_good:
                if mem.good_rearm < 50:
                    mem.good_rearm += 1
            else:
                if mem.good_rearm > 0:
                    mem.good_rearm -= 1

            can_detect = (
                mem.events_armed
                and (ticks_diff(t_now, mem.last_event_t) >= config.EVENT_COOLDOWN_MS)
                and (mem.good_rearm >= config.EVENT_REARM_N)
            )

            if can_detect and event_cond:
                mem.event_in_count += 1
                if mem.event_in_count >= config.EVENT_ENTER_N:
                    mem.event_in_count = 0
                    mem.last_event_t = t_now
                    mem.good_rearm = 0

                    step = nav.get_step(mission, mem.step)
                    if step is None:
                        continue

                    scan = step.get("scan")
                    if (
                        nav.mode in ("scan", "resume_scan")
                        and scan is not None
                        and nav.slot_needs_scan(scan["stack"], scan["slot"])
                    ):
                        has_reel = task_sensors.branch_has_reel(
                            scan["stack"],
                            scan["slot"],
                            scan["side"],
                        )
                        dbg("SCAN CHECK", scan, "has_reel=", has_reel)

                        if has_reel:
                            nav.register_reel_found(
                                scan["stack"],
                                scan["slot"],
                                scan["turn"],
                                step["node"],
                                step["heading_out"],
                            )
                            mem.grabbed_colour = None
                            mem.dir_turn = scan["turn"]
                            set_state(mem, states.GRAB, "reel detected -> grab")
                            continue
                        else:
                            nav.mark_slot_checked(scan["stack"], scan["slot"])

                    act = step["move"]
                    dbg(
                        "ACTION",
                        act,
                        "node=", step["node"],
                        "step=", mem.step,
                        "spin_dir=", step.get("spin_dir", None),
                    )
                    if not _dispatch_action(mem, states, step, inter_cond):
                        set_state(mem, states.STOP, "bad action {}".format(act))
                        continue

            elif not event_cond:
                mem.event_in_count = 0

            if err is None:
                if inter_cond or sumw >= 3:
                    motors.arcade(config.STRAIGHT_THROTTLE, 0.0)
                else:
                    motors.arcade(
                        config.SEARCH_THROTTLE,
                        mem.last_search_dir * config.SEARCH_STEER,
                    )
                continue

            thr, steer = pd_follow(err, mem.last_err, dt_s)
            motors.arcade(thr, steer)
            mem.last_err = err
            continue

        if mem.state == states.DO_STRAIGHT:
            straight_movement(mem, motors, sensors, states, mission)
            continue

        if mem.state in (states.TURN_APPROACH, states.TURN_SPIN, states.TURN_ALIGN):
            turning_movement(mem, motors, sensors, states)
            continue

        if mem.state in (states.SPIN180_SPIN, states.SPIN180_ALIGN):
            spin180_movement(mem, motors, sensors, states)
            continue

        if mem.state == states.GRAB:
            grab_movement(mem, motors, sensors, states, task_sensors, nav.active_stack)
            if mem.state == states.FOLLOW:
                if mem.grabbed_colour is None:
                    set_state(mem, states.STOP, "grab completed but no colour identified")
                    continue

                mission = nav.build_delivery_mission(mem.grabbed_colour)
                mem.step = 0
                _reset_after_macro(mem)
            continue

        if mem.state == states.PLACE:
            colour = nav.current_colour
            place_cfg = nav.place_cfg[colour]
            place_movement(mem, motors, sensors, states, task_sensors, colour, place_cfg)

            if mem.state == states.FOLLOW:
                nav.set_post_place_pose_from_current_colour()

                projected_total = nav.total_delivered() + 1

                if projected_total >= nav.expected_total_reels:
                    mission = nav.build_return_home_mission()
                    next_mode = "return_home"
                else:
                    mission = nav.build_resume_scan_mission()
                    next_mode = "resume_scan"

                nav.complete_delivery_cycle()
                nav.mode = next_mode
                nav.current_mission = mission

                mem.step = 0
                _reset_after_macro(mem)
            continue

        set_state(mem, states.STOP, "fell through state machine")


main()