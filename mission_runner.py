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
from control.movement import border_push_movement, turning_movement
from control.pd import middle_error_white_line, pd_follow

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

# ---------------- Main runner ----------------
def main():
    # sensors + motors
    sensors = LineSensors(config.LINE_PINS, invert=config.LINE_INVERT)

    left = DCMotor(config.MOTOR_L_DIR,
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
    mission = build_mission()
    mem = Mem() 
    states = State()
    t_last = ticks_ms()

    border_push_movement(mem,motors,sensors,states)

    while True:

        # Timing: enforce fixed loop rate and get current time
        t_now, t_last = fixed_rate_tick(t_last, period_ms)

        # Sense
        black, white, sumw, good_line, inter_cond, corner_raw = sensors.sense()

        # Event logic (computed always; used mainly in FOLLOW and a few heuristics)
        inter_cond, corner_cond, event_cond, recent_good = compute_event(t_now, mem, corner_raw, inter_cond)

        if good_line:
            mem.t_last_good_line = t_now
        err = middle_error_white_line(black)

        # ---------------- STOP ----------------
        if mem.state == states.STOP:
            motors.arcade(0.0, 0.0)
            break

        # ---------------- FOLLOW ----------------
        if mem.state == states.FOLLOW:
            if mem.step >= len(mission):
                set_state(mem, states.FINAL_FOLLOW, "mission complete -> final follow")
                continue

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

            can_detect = (mem.events_armed and (ticks_diff(t_now, mem.last_event_t) >= config.EVENT_COOLDOWN_MS)and (mem.good_rearm >= config.EVENT_REARM_N)
                          
            )

            if can_detect:
                if event_cond:
                    mem.event_in_count += 1
                    if mem.event_in_count >= config.EVENT_ENTER_N:
                        mem.event_in_count = 0
                        mem.last_event_t = t_now
                        mem.good_rearm = 0

                        act = expected_action(mission, mem.step)
                        if act is None:
                            set_state(mem, states.FINAL_FOLLOW, "mission complete -> final follow")
                            continue

                        # If we're about to execute the last scripted action, enter finish mode.
                        if mem.step == (len(mission) - 1):
                            mem.finish_mode = True

                        mem.last_event_was_intersection = bool(inter_cond)

                        if act == "straight":
                            mem.straight_out = 0
                            set_state(mem, states.DO_STRAIGHT, "event->straight")
                            continue

                        if act == "left":
                            mem.dir_turn = -1
                            if mem.last_event_was_intersection:
                                set_state(mem, states.TURN_APPROACH, "event->left (approach)")
                            else:
                                set_state(mem, states.TURN_SPIN, "event->left (corner)")
                            continue

                        if act == "right":
                            mem.dir_turn = +1
                            if mem.last_event_was_intersection:
                                set_state(mem, states.TURN_APPROACH, "event->right (approach)")
                            else:
                                set_state(mem, states.TURN_SPIN, "event->right (corner)")
                            continue

                        set_state(mem, states.STOP, "bad action {}".format(act))
                        continue
                else:
                    mem.event_in_count = 0

            # lost handling: never freeze
            if err is None:
                if inter_cond or sumw >= 3:
                    motors.arcade(config.STRAIGHT_THROTTLE, 0.0)
                else:
                    motors.arcade(config.SEARCH_THROTTLE, mem.last_search_dir * config.SEARCH_STEER)
                continue

            # normal PD follow
            thr, steer = pd_follow(err, mem.last_err, dt_s)
            motors.arcade(thr, steer)
            mem.last_err = err
            continue

        # ---------------- DO_STRAIGHT ----------------
        if mem.state == states.DO_STRAIGHT:
            elapsed = ticks_diff(t_now, mem.state_t0)

            steer = 0.0
            if err is not None:
                steer = clamp(config.STRAIGHT_KP * err, -config.STRAIGHT_MAX_STEER, +config.STRAIGHT_MAX_STEER)
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

        # ---------------- TURN ----------------
        if mem.state == states.TURN_SPIN:
            turning_movement(mem,motors,sensors,states)
            continue


main()
