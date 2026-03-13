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
from hw.task_sensors_test import TaskSensors


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
    
def _dispatch_current_step_immediately(mem, states, nav, mission):
    step = nav.get_step(mission, mem.step)
    if step is None:
        return False

    act = step["move"]
    dbg(
        "IMMEDIATE ACTION",
        act,
        "node=", step["node"],
        "step=", mem.step,
        "spin_dir=", step.get("spin_dir", None),
        "heading_in=", step.get("heading_in", None),
        "heading_out=", step.get("heading_out", None),
    )

    return _dispatch_action(mem, states, step, inter_cond=False)

def _reset_after_macro(mem):
    dbg("RESET AFTER MACRO", "old_step=", mem.step, "old_state=", mem.state)
    mem.last_event_t = ticks_ms()
    mem.good_rearm = 0
    mem.event_in_count = 0
    mem.last_err = 0.0
    dbg(
        "RESET DONE",
        "last_event_t=", mem.last_event_t,
        "good_rearm=", mem.good_rearm,
        "event_in_count=", mem.event_in_count,
        "last_err=", mem.last_err,
    )


def _build_initial_mission(nav):
    mission = nav.build_initial_scan_mission()
    dbg("INITIAL SCAN CAMPAIGN BUILT")
    dbg(
        "INITIAL MISSION",
        "steps=", len(mission["steps"]),
        "end_node=", mission["end_node"],
        "end_heading=", mission["end_heading"],
    )
    first = nav.get_step(mission, 0)
    dbg("INITIAL STEP 0 =", first)
    return mission


def _maybe_wait_for_start(task_sensors, motors):
    dbg("WAITING FOR START BUTTON")
    while not task_sensors.start_pressed():
        motors.arcade(0.0, 0.0)
        sleep_ms(20)
    dbg("START BUTTON PRESSED")


def _dispatch_action(mem, states, step, inter_cond):
    act = step["move"]

    dbg(
        "DISPATCH",
        "act=", act,
        "node=", step["node"],
        "step=", mem.step,
        "inter_cond=", inter_cond,
        "state_before=", mem.state,
    )

    if act == "straight":
        mem.straight_out = 0
        set_state(mem, states.DO_STRAIGHT, "event->straight")
        dbg("DISPATCH RESULT", "new_state=", mem.state, "straight_out=", mem.straight_out)
        return True

    if act == "left":
        mem.dir_turn = -1
        if inter_cond:
            set_state(mem, states.TURN_APPROACH, "event->left (approach)")
        else:
            set_state(mem, states.TURN_SPIN, "event->left (corner)")
        dbg("DISPATCH RESULT", "new_state=", mem.state, "dir_turn=", mem.dir_turn)
        return True

    if act == "right":
        mem.dir_turn = +1
        if inter_cond:
            set_state(mem, states.TURN_APPROACH, "event->right (approach)")
        else:
            set_state(mem, states.TURN_SPIN, "event->right (corner)")
        dbg("DISPATCH RESULT", "new_state=", mem.state, "dir_turn=", mem.dir_turn)
        return True

    if act == "180":
        mem.dir_turn = step.get("spin_dir", +1)
        set_state(mem, states.SPIN180_SPIN, "event->180")
        dbg("DISPATCH RESULT", "new_state=", mem.state, "dir_turn=", mem.dir_turn)
        return True

    dbg("DISPATCH FAILED", "bad action=", act)
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

    dbg("BOOT", "period_ms=", period_ms, "dt_s=", dt_s)
    if hasattr(states, "__dict__"):
        dbg("STATE ENUMS", states.__dict__)
    else:
        dbg("STATE ENUMS OBJECT", states)

    t_last = ticks_ms()
    last_dbg_t = t_last

    _maybe_wait_for_start(task_sensors, motors)

    mission = _build_initial_mission(nav)

    dbg("ENTER BORDER PUSH", "state_before=", mem.state)
    border_push_movement(mem, motors, sensors, states)
    dbg("EXIT BORDER PUSH", "state_after=", mem.state)

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
            dbg("STOP REQUEST", "reason=external stop button")
            set_state(mem, states.STOP, "external stop pressed")

        if mem.state == states.STOP:
            dbg("MAIN BREAK", "reason=STOP state reached", "step=", mem.step, "mode=", nav.mode)
            motors.arcade(0.0, 0.0)
            break

        if mem.state == states.FOLLOW:
            step = nav.get_step(mission, mem.step)

            if step is None:
                dbg("MISSION EXHAUSTED", "mode=", nav.mode, "step=", mem.step)

                if nav.mode in ("scan", "resume_scan"):
                    if nav.all_resolved():
                        dbg("ALL RESOLVED -> BUILD RETURN HOME")
                        mission = nav.build_return_home_mission()
                        dbg(
                            "RETURN HOME BUILT",
                            "steps=", len(mission["steps"]),
                            "end_node=", mission["end_node"],
                            "end_heading=", mission["end_heading"],
                        )
                        mem.step = 0
                        _reset_after_macro(mem)
                    else:
                        dbg("STOP REQUEST", "reason=scan exhausted before all reels delivered")
                        set_state(
                            mem,
                            states.STOP,
                            "scan campaign exhausted before all expected reels were delivered",
                        )
                    continue

                if nav.mode == "to_place_approach":
                    dbg("ARRIVED AT PLACE APPROACH", "node=", nav.place_approach_node)
                    set_state(
                        mem,
                        states.PLACE,
                        "arrived at bay approach {}".format(nav.place_approach_node),
                    )
                    continue

                if nav.mode == "return_home":
                    dbg("STOP REQUEST", "reason=all reels delivered; back at node 1")
                    set_state(mem, states.STOP, "all reels delivered; back at node 1")
                    continue

                dbg("STOP REQUEST", "reason=unknown nav mode", nav.mode)
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
                dbg(
                    "EVENT COUNTING",
                    "count=", mem.event_in_count,
                    "need=", config.EVENT_ENTER_N,
                    "node_step=", mem.step,
                )

                if mem.event_in_count >= config.EVENT_ENTER_N:
                    mem.event_in_count = 0
                    mem.last_event_t = t_now
                    mem.good_rearm = 0

                    step = nav.get_step(mission, mem.step)
                    if step is None:
                        dbg("EVENT FIRED BUT STEP IS NONE", "step=", mem.step)
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
                            dbg(
                                "REEL DETECTED",
                                "stack=", scan["stack"],
                                "slot=", scan["slot"],
                                "turn=", scan["turn"],
                                "node=", step["node"],
                                "heading_out=", step["heading_out"],
                            )
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
                            dbg("MARK SLOT CHECKED", "stack=", scan["stack"], "slot=", scan["slot"])
                            nav.mark_slot_checked(scan["stack"], scan["slot"])

                    act = step["move"]
                    dbg(
                        "ACTION",
                        act,
                        "node=", step["node"],
                        "step=", mem.step,
                        "spin_dir=", step.get("spin_dir", None),
                        "heading_in=", step.get("heading_in", None),
                        "heading_out=", step.get("heading_out", None),
                    )
                    if not _dispatch_action(mem, states, step, inter_cond):
                        dbg("STOP REQUEST", "reason=bad action", act)
                        set_state(mem, states.STOP, "bad action {}".format(act))
                        continue

            elif not event_cond:
                if mem.event_in_count != 0:
                    dbg("EVENT COUNT RESET", "old_count=", mem.event_in_count)
                mem.event_in_count = 0

            if err is None:
                if inter_cond or sumw >= 3:
                    dbg("FOLLOW DRIVE", "mode=blind-straight", "thr=", config.STRAIGHT_THROTTLE, "steer=0.0")
                    motors.arcade(config.STRAIGHT_THROTTLE, 0.0)
                else:
                    dbg(
                        "FOLLOW DRIVE",
                        "mode=search",
                        "thr=", config.SEARCH_THROTTLE,
                        "steer=", mem.last_search_dir * config.SEARCH_STEER,
                    )
                    motors.arcade(
                        config.SEARCH_THROTTLE,
                        mem.last_search_dir * config.SEARCH_STEER,
                    )
                continue

            thr, steer = pd_follow(err, mem.last_err, dt_s)
            dbg("FOLLOW DRIVE", "mode=pd", "thr=", thr, "steer=", steer, "err=", err, "last_err=", mem.last_err)
            motors.arcade(thr, steer)
            mem.last_err = err
            continue

        if mem.state == states.DO_STRAIGHT:
            dbg("ENTER MACRO", "macro=straight", "state=", mem.state, "step=", mem.step)
            straight_movement(mem, motors, sensors, states, mission)
            dbg("EXIT MACRO", "macro=straight", "state=", mem.state, "step=", mem.step)
            if mem.state == states.DO_STRAIGHT:
                dbg("WARNING", "straight_movement returned but state still DO_STRAIGHT")
            if mem.state == states.STOP:
                dbg("STRAIGHT CAUSED STOP", "step=", mem.step)
            continue

        if mem.state in (states.TURN_APPROACH, states.TURN_SPIN, states.TURN_ALIGN):
            dbg("ENTER MACRO", "macro=turn", "state=", mem.state, "step=", mem.step, "dir_turn=", mem.dir_turn)
            turning_movement(mem, motors, sensors, states)
            dbg("EXIT MACRO", "macro=turn", "state=", mem.state, "step=", mem.step)
            if mem.state in (states.TURN_APPROACH, states.TURN_SPIN, states.TURN_ALIGN):
                dbg("WARNING", "turning_movement returned but still in turn state")
            if mem.state == states.STOP:
                dbg("TURN CAUSED STOP", "step=", mem.step)
            continue

        if mem.state in (states.SPIN180_SPIN, states.SPIN180_ALIGN):
            dbg("ENTER MACRO", "macro=spin180", "state=", mem.state, "step=", mem.step, "dir_turn=", mem.dir_turn)
            spin180_movement(mem, motors, sensors, states)
            dbg("EXIT MACRO", "macro=spin180", "state=", mem.state, "step=", mem.step)
            if mem.state in (states.SPIN180_SPIN, states.SPIN180_ALIGN):
                dbg("WARNING", "spin180_movement returned but still in 180 state")
            if mem.state == states.STOP:
                dbg("SPIN180 CAUSED STOP", "step=", mem.step)
            continue

        if mem.state == states.GRAB:
            dbg("ENTER MACRO", "macro=grab", "state=", mem.state, "step=", mem.step, "stack=", nav.active_stack)
            grab_movement(mem, motors, sensors, states, task_sensors, nav.active_stack)
            dbg(
                "EXIT MACRO",
                "macro=grab",
                "state=", mem.state,
                "step=", mem.step,
                "grabbed_colour=", mem.grabbed_colour,
            )
            if mem.state == states.FOLLOW:
                if mem.grabbed_colour is None:
                    dbg("STOP REQUEST", "reason=grab completed but no colour identified")
                    set_state(mem, states.STOP, "grab completed but no colour identified")
                    continue

                dbg("BUILD DELIVERY MISSION", "colour=", mem.grabbed_colour)
                mission = nav.build_delivery_mission(mem.grabbed_colour)
                dbg(
                    "DELIVERY MISSION BUILT",
                    "steps=", len(mission["steps"]),
                    "end_node=", mission["end_node"],
                    "end_heading=", mission["end_heading"],
                )
                mem.step = 0
                _reset_after_macro(mem)
                if not _dispatch_current_step_immediately(mem, states, nav, mission):
                    set_state(mem, states.STOP, "failed to dispatch first delivery step")

            elif mem.state == states.STOP:
                dbg("GRAB CAUSED STOP", "step=", mem.step)
            continue

        if mem.state == states.PLACE:
            colour = nav.current_colour
            place_cfg = nav.place_cfg[colour]

            dbg(
                "ENTER MACRO",
                "macro=place",
                "state=", mem.state,
                "step=", mem.step,
                "colour=", colour,
                "place_cfg=", place_cfg,
            )

            place_movement(mem, motors, sensors, states, task_sensors, colour, place_cfg)

            dbg("EXIT MACRO", "macro=place", "state=", mem.state, "step=", mem.step)

            if mem.state == states.FOLLOW:
                nav.set_post_place_pose_from_current_colour()
                dbg(
                    "POST PLACE POSE",
                    "node=", nav.post_place_node,
                    "heading=", nav.post_place_heading,
                )

                projected_total = nav.total_delivered() + 1
                dbg(
                    "DELIVERY PROJECTION",
                    "projected_total=", projected_total,
                    "expected_total=", nav.expected_total_reels,
                )

                if projected_total >= nav.expected_total_reels:
                    dbg("BUILD RETURN HOME AFTER PLACE")
                    mission = nav.build_return_home_mission()
                    next_mode = "return_home"
                else:
                    dbg("BUILD RESUME SCAN AFTER PLACE")
                    mission = nav.build_resume_scan_mission()
                    next_mode = "resume_scan"

                dbg(
                    "POST-PLACE MISSION BUILT",
                    "mode=", next_mode,
                    "steps=", len(mission["steps"]),
                    "end_node=", mission["end_node"],
                    "end_heading=", mission["end_heading"],
                )

                nav.complete_delivery_cycle()
                nav.mode = next_mode
                nav.current_mission = mission

                mem.step = 0
                _reset_after_macro(mem)
                if not _dispatch_current_step_immediately(mem, states, nav, mission):
                    set_state(mem, states.STOP, "failed to dispatch first post-place step")
                    
            elif mem.state == states.STOP:
                dbg("PLACE CAUSED STOP", "step=", mem.step)

            continue

        dbg("STOP REQUEST", "reason=fell through state machine", "state=", mem.state)
        set_state(mem, states.STOP, "fell through state machine")


main()