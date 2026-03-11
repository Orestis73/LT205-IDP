# src/mission_runner.py
#
# Real course runner (not a test). No getattr fallbacks:
# everything comes from config.py. Mission comes from src/mission.py.

from utime import ticks_ms, ticks_diff, sleep_ms

import config
from nav.mission import build_mission
from hw.line import LineSensors
from hw.motors import DCMotor, MotorPair
from control.ulilities import clamp, b4
from control.runtime_variables import Mem, State
from control.ulilities import set_state
from control.movement import (
    border_push_movement,
    spin180_movement,
    turning_movement,
    straight_movement,
    grab_movement,
)
from control.pd import middle_error_white_line, pd_follow
from state_machine_1 import navigation


# ---------------- Debug ----------------
DEBUG = True
DEBUG_LOOP_MS = 200   # periodic loop print to avoid total spam


def dbg(*args):
    if DEBUG:
        print("[DBG]", *args)


def fixed_rate_tick(t_last, period_ms):
    """Returns (t_now, new_t_last)."""
    t_now = ticks_ms()
    dt = ticks_diff(t_now, t_last)
    if dt < period_ms:
        sleep_ms(period_ms - dt)
        t_now = ticks_ms()
    return t_now, t_now


def expected_action(mission, step):
    return mission[step] if step < len(mission) else None


def compute_event(t_now, mem, corner_raw, inter_cond):
    # recent good line gate
    recent_good = ticks_diff(t_now, mem.t_last_good_line) <= config.RECENT_GOOD_LINE_MS

    # corner only counts if we were on the line recently
    corner_cond = recent_good and corner_raw and (not inter_cond)

    # event condition: either effective intersection or corner
    event_cond = inter_cond or corner_cond

    return inter_cond, corner_cond, event_cond, recent_good


def get_new_mission(mem, nav):
    mem.step = 0
    mission = nav.route_executor()
    dbg("NEW MISSION:", mission, "| nav_state =", nav.nav_state, "| scanning =", nav.scanning)
    return mission


# ---------------- Main runner ----------------
def main():
    # sensors + motors
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

    # timing
    period_ms = int(1000 / config.LOOP_HZ)
    dt_s = period_ms / 1000.0

    # mission
    mem = Mem()
    nav = navigation()
    states = State()

    t_last = ticks_ms()
    last_dbg_t = t_last

    dbg("START nav_state =", nav.nav_state)
    mission = get_new_mission(mem, nav)

    dbg("ENTER border_push")
    border_push_movement(mem, motors, sensors, states)
    dbg("EXIT border_push | state =", mem.state)

    dbg("AFTER BORDER PUSH | nav_state =", nav.nav_state, "| mission =", mission)

    while True:
        # Timing: enforce fixed loop rate and get current time
        t_now, t_last = fixed_rate_tick(t_last, period_ms)

        # Sense
        black, white, sumw, good_line, inter_raw, corner_raw = sensors.sense()

        if good_line:
            mem.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(
            t_now, mem, corner_raw, inter_raw
        )

        err = middle_error_white_line(black)

        # periodic loop debug
        if ticks_diff(t_now, last_dbg_t) >= DEBUG_LOOP_MS:
            dbg(
                "LOOP",
                "state=", mem.state,
                "step=", mem.step,
                "nav=", nav.nav_state,
                "scan=", nav.scanning,
                "sumw=", sumw,
                "black=", b4(black),
                "white=", b4(white),
                "good=", good_line,
                "recent_good=", recent_good,
                "inter=", inter_cond,
                "corner=", corner_cond,
                "event=", event_cond,
                "err=", err,
                "armed=", mem.events_armed,
                "rearm=", mem.good_rearm,
                "event_in_count=", mem.event_in_count,
            )
            last_dbg_t = t_now

        # ---------------- STOP ----------------
        if mem.state == states.STOP:
            dbg("STOP reached")
            motors.arcade(0.0, 0.0)
            break

        # ---------------- FOLLOW ----------------
        if mem.state == states.FOLLOW:
            if mem.step >= len(mission):
                dbg("MISSION EXHAUSTED | requesting new mission")
                mission = get_new_mission(mem, nav)

            if mission == ["END"]:
                dbg("MISSION IS ['END'] -> STOP")
                set_state(mem, states.STOP)
                continue

            # IMPORTANT:
            # the unconditional `continue` that used to be here made all follow logic below unreachable

            # update last_search_dir memory
            if err is not None:
                if err > 0:
                    mem.last_search_dir = +1
                elif err < 0:
                    mem.last_search_dir = -1

            # rearm counter (use "good line recently")
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

            if event_cond:
                dbg(
                    "EVENT SEEN",
                    "inter=", inter_cond,
                    "corner=", corner_cond,
                    "recent_good=", recent_good,
                    "can_detect=", can_detect,
                )

            if can_detect:
                if event_cond:
                    mem.event_in_count += 1
                    dbg("EVENT COUNT ++ ->", mem.event_in_count, "/", config.EVENT_ENTER_N)

                    if mem.event_in_count >= config.EVENT_ENTER_N:
                        mem.event_in_count = 0
                        mem.last_event_t = t_now
                        mem.good_rearm = 0

                        act = expected_action(mission, mem.step)
                        next_act = expected_action(mission, mem.step + 1)

                        dbg(
                            "EVENT CONFIRMED",
                            "step=", mem.step,
                            "act=", act,
                            "next_act=", next_act,
                            "mission=", mission,
                            "nav_state=", nav.nav_state,
                            "scanning=", nav.scanning,
                            "current_stack=", nav.current_stack,
                        )

                        if nav.scanning:
                            nav.stack_decider()
                            nav.stack_reel_count[nav.current_stack][0] += 1
                            dbg(
                                "SCANNING UPDATE",
                                "stack=", nav.current_stack,
                                "count=", nav.stack_reel_count[nav.current_stack],
                            )

                        if nav.stack_reel_count[nav.current_stack][1] and nav.scanning:
                            # TO BE REPLACED BY ACTUAL DETECTION LOGIC
                            if nav.stack_reel_count[nav.current_stack][1] <= nav.stack_reel_count[nav.current_stack][0]:
                                nav.scanning = False
                                nav.nav_state = "picking"
                                nav.destination_decider()
                                dbg(
                                    "REEL FOUND",
                                    "stack=", nav.current_stack,
                                    "nav_state ->", nav.nav_state,
                                )
                                mission = get_new_mission(mem, nav)
                                set_state(mem, states.GRAB, "Reel found -> Starting full grab sequence")
                                continue

                        if next_act == "start_scan" or act=="start_scan":
                            nav.scanning = True
                            mem.step += 1
                            dbg("NEXT ACTION start_scan -> scanning=True, step=", mem.step)

                        if next_act == "end_scan":
                            nav.scanning = False
                            mem.step += 1
                            dbg("NEXT ACTION end_scan -> scanning=False, step=", mem.step)

                        if act is None:
                            dbg("ACT IS NONE -> STOP")
                            set_state(mem, states.STOP, "mission complete -> stop")
                            continue

                        if act == "straight":
                            dbg("ACT -> STRAIGHT")
                            mem.straight_out = 0
                            set_state(mem, states.DO_STRAIGHT, "event->straight")
                            continue

                        if act == "left":
                            dbg("ACT -> LEFT | inter_cond =", inter_cond)
                            mem.dir_turn = -1
                            if inter_cond:
                                set_state(mem, states.TURN_APPROACH, "event->left (approach)")
                            else:
                                set_state(mem, states.TURN_SPIN, "event->left (corner)")
                            continue

                        if act == "right":
                            dbg("ACT -> RIGHT | inter_cond =", inter_cond)
                            mem.dir_turn = +1
                            if inter_cond:
                                set_state(mem, states.TURN_APPROACH, "event->right (approach)")
                            else:
                                set_state(mem, states.TURN_SPIN, "event->right (corner)")
                            continue

                        if act == "180":
                            dbg("ACT -> 180")
                            mem.dir_turn = +1
                            set_state(mem, states.SPIN180_SPIN, "event->180")
                            continue

                        dbg("BAD ACTION:", act)
                        set_state(mem, states.STOP, "bad action {}".format(act))
                        continue
                else:
                    if mem.event_in_count != 0:
                        dbg("EVENT COUNT RESET")
                    mem.event_in_count = 0

            # lost handling: never freeze
            if err is None:
                if inter_cond or sumw >= 3:
                    dbg("LOST LINE but broad white/intersection -> straight crawl")
                    motors.arcade(config.STRAIGHT_THROTTLE, 0.0)
                else:
                    dbg(
                        "LOST LINE -> searching",
                        "dir=", mem.last_search_dir,
                        "thr=", config.SEARCH_THROTTLE,
                        "steer=", mem.last_search_dir * config.SEARCH_STEER,
                    )
                    motors.arcade(
                        config.SEARCH_THROTTLE,
                        mem.last_search_dir * config.SEARCH_STEER
                    )
                continue

            # normal PD follow
            thr, steer = pd_follow(err, mem.last_err, dt_s)
            dbg("PD", "err=", err, "last_err=", mem.last_err, "thr=", thr, "steer=", steer)
            motors.arcade(thr, steer)
            mem.last_err = err
            continue

        # ---------------- DO_STRAIGHT ----------------
        if mem.state == states.DO_STRAIGHT:
            dbg("ENTER DO_STRAIGHT")
            straight_movement(mem, motors, sensors, states, mission)
            continue

        # ---------------- TURN ----------------
        if mem.state in (states.TURN_APPROACH, states.TURN_SPIN, states.TURN_ALIGN):
            dbg("ENTER TURN STATE", mem.state)
            turning_movement(mem, motors, sensors, states)
            continue

        # ---------------- 180 ----------------
        if mem.state in (states.SPIN180_SPIN, states.SPIN180_ALIGN):
            dbg("ENTER SPIN180 STATE", mem.state)
            spin180_movement(mem, motors, sensors, states)
            continue

        # ---------------- GRAB ----------------
        if mem.state == states.GRAB:
            nav.stack_decider()
            dbg("ENTER GRAB | current_stack =", nav.current_stack)

            if nav.current_stack in ["pd", "od", "ou"]:
                mem.dir_turn = -1
            if nav.current_stack == "pu":
                mem.dir_turn = +1

            dbg("GRAB dir_turn =", mem.dir_turn)
            grab_movement(mem, motors, sensors, states)
            continue


main()
