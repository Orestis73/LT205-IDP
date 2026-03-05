# src/mission_runner.py
#
# Real course runner (not a test). No getattr fallbacks:
# everything comes from config.py. Mission comes from src/mission.py.

from utime import ticks_ms, ticks_diff, sleep_ms

import config
from nav.mission import build_mission
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


# ---------------- State labels ----------------
START_BOX = 0
BORDER_PUSH = 1
ACQUIRE = 2
GATE_CLEAR = 3
ARM_WAIT = 4
FOLLOW = 5
DO_STRAIGHT = 6
TURN_APPROACH = 7
TURN_SPIN = 8
TURN_ALIGN = 9
FINAL_FOLLOW = 10
FINAL_FORWARD = 11
STOP = 12


# For debug printing 
STATE_NAMES = {
    START_BOX: "START_BOX",
    BORDER_PUSH: "BORDER_PUSH",
    ACQUIRE: "ACQUIRE",
    GATE_CLEAR: "GATE_CLEAR",
    ARM_WAIT: "ARM_WAIT",
    FOLLOW: "FOLLOW",
    DO_STRAIGHT: "DO_STRAIGHT",
    TURN_APPROACH: "TURN_APPROACH",
    TURN_SPIN: "TURN_SPIN",
    TURN_ALIGN: "TURN_ALIGN",
    FINAL_FOLLOW: "FINAL_FOLLOW",
    FINAL_FORWARD: "FINAL_FORWARD",
    STOP: "STOP",
}


class Mem:
    """All mutable runtime variables in one place."""

    def __init__(self):
        self.state = START_BOX
        self.state_t0 = ticks_ms()

        self.step = 0
        self.finish_mode = False

        self.last_event_t = 0
        self.events_armed = False

        self.last_err = 0.0
        self.last_search_dir = +1

        self.see_white_count = 0
        self.border_ok = 0
        self.acquire_ok = 0
        self.good_rearm = 0

        self.event_in_count = 0
        self.straight_out = 0

        self.dir_turn = +1
        self.t_last_good_line = ticks_ms()

        self.t_last_inter = ticks_ms() - 10_000
        self.last_event_was_intersection = False

        self.dbg_next = ticks_ms()


def set_state(mem, new_state, why=""):
    mem.state = new_state
    mem.state_t0 = ticks_ms()
    if config.DEBUG:
        if why:
            print("STATE ->", STATE_NAMES.get(new_state, new_state), "|", why)
        else:
            print("STATE ->", STATE_NAMES.get(new_state, new_state))


def fixed_rate_tick(t_last, period_ms):
    """Returns (t_now, new_t_last)."""
    t_now = ticks_ms()
    dt = ticks_diff(t_now, t_last)
    if dt < period_ms:
        sleep_ms(period_ms - dt)
        t_now = ticks_ms()
    return t_now, t_now


def sense(sensors, t_now, mem):
    black = sensors.read_black()
    white = [1 - b for b in black]  # 1=WHITE
    sumw = white[0] + white[1] + white[2] + white[3]
    err = middle_error_white_line(black)

    good_line = (white[0] == 0 and white[1] == 1 and white[2] == 1 and white[3] == 0)
    if good_line:
        mem.t_last_good_line = t_now

    return black, white, sumw, err, good_line

"""
def compute_event(t_now, white, sumw, mem):
    # Intersection is a "fat" patch: many whites or both outers, etc.
    inter_cond = (
        (sumw >= config.INTER_SUMW)
        or (white[0] and white[3])
       # or ((white[0] or white[3]) and (white[1] or white[2]))
    )
    if inter_cond:
        mem.t_last_inter = t_now

    inter_recent = ticks_diff(t_now, mem.t_last_inter) <= config.INTER_LATCH_MS
    recent_good = ticks_diff(t_now, mem.t_last_good_line) <= config.RECENT_GOOD_LINE_MS

    # Corner only if we haven't *recently* seen an intersection.
    corner_cond = (
        (not inter_recent)
        and recent_good
        and (white[1] == 0 and white[2] == 0)
        and ((white[0] ^ white[3]) == 1)
    )

    # Effective event type: treat anything within latch window as intersection.
    eff_inter = inter_cond or inter_recent
    event_cond = eff_inter or corner_cond

    return inter_cond, eff_inter, corner_cond, event_cond, recent_good

"""


def compute_event(t_now, white, sumw, mem):
 
    # --- recent good line gate (keeps noise from firing events when lost) ---
    recent_good = ticks_diff(t_now, mem.t_last_good_line) <= config.RECENT_GOOD_LINE_MS

    # --- Intersection: both outers white ---
    inter_cond = bool(white[0] and white[3])

    # --- Corner-like (your case 1/2 patterns): exactly one outer is 0 AND sumw==3 ---
    # This matches 0111 or 1110, and avoids 1111 (intersection) and avoids 0110 (normal line).
    corner_raw = (sumw == 3) and ((white[0] ^ white[3]) == 1)

    # Optional: gate corners by "we were on line recently" so random junk doesn't trigger it.
    corner_cond = recent_good and corner_raw and (not inter_cond)

    # --- latch intersection so we don't mis-handle flicker around it ---
    if inter_cond:
        mem.t_last_inter = t_now
    inter_recent = ticks_diff(t_now, mem.t_last_inter) <= config.INTER_LATCH_MS

    # Effective intersection: intersection now OR within latch window
    eff_inter = inter_cond or inter_recent

    # Event: either type
    event_cond = eff_inter or corner_cond

    return inter_cond, eff_inter, corner_cond, event_cond, recent_good




def pd_follow(err, last_err, dt_s):
    derr = (err - last_err) / dt_s
    steer = config.KP * err + config.KD * derr
    steer = clamp(steer, -config.MAX_STEER, +config.MAX_STEER)

    throttle = config.BASE_THROTTLE - config.SLOW_K * abs(steer)
    throttle = clamp(throttle, config.MIN_THROTTLE, config.BASE_THROTTLE)
    return throttle, steer


def expected_action(mission, step):
    return mission[step] if step < len(mission) else None


def dbg_print(t_now, mem, mission, sumw, white, err, good_line, event_cond, eff_inter, corner_cond):
    if not config.DEBUG:
        return

    if ticks_diff(t_now, mem.dbg_next) >= config.DEBUG_MS:
        mem.dbg_next = t_now
        print(
            "DBG",
            "S=", STATE_NAMES.get(mem.state, mem.state),
            "step=", mem.step,
            "act=", expected_action(mission, mem.step),
            "sumw=", sumw,
            "w=", b4(white),
            "err=", err,
            "good=", 1 if good_line else 0,
            "ev=", 1 if event_cond else 0,
            "inter=", 1 if eff_inter else 0,
            "corner=", 1 if corner_cond else 0,
            "armed=", 1 if mem.events_armed else 0,
            "rearm=", mem.good_rearm,
            "cool=", ticks_diff(t_now, mem.last_event_t),
        )


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
    mission = build_mission()
    main_mission = mission  # for dbg_print access
    mission_runner.main_mission = mission

    mem = Mem()

    print("BOOT: mission length =", len(mission))
    if config.DEBUG:
        print("STATE ->", STATE_NAMES.get(mem.state, mem.state))

    t_last = ticks_ms()

    while True:
        t_now, t_last = fixed_rate_tick(t_last, period_ms)

        # Sense
        black, white, sumw, err, good_line = sense(sensors, t_now, mem)

        # Event logic (computed always; used mainly in FOLLOW and a few heuristics)
        inter_cond, eff_inter, corner_cond, event_cond, recent_good = compute_event(t_now, white, sumw, mem)

        # Debug
        dbg_print(t_now, mem, mission, sumw, white, err, good_line, event_cond, eff_inter, corner_cond)


        # ---------------- STOP ----------------
        if mem.state == STOP:
            motors.arcade(0.0, 0.0)
            continue

        # ---------------- START_BOX ----------------
        if mem.state == START_BOX:
            motors.arcade(config.START_THROTTLE, 0.0)
            if sumw >= 1:
                mem.see_white_count += 1
                if mem.see_white_count >= config.START_SEE_WHITE_N:
                    mem.see_white_count = 0
                    mem.border_ok = 0
                    set_state(mem, BORDER_PUSH, "saw white at start, pushing border")
            else:
                mem.see_white_count = 0
            continue

        """
        # ---------------- BORDER_PUSH ----------------
        if mem.state == BORDER_PUSH:
            if ticks_diff(t_now, mem.state_t0) >= config.BORDER_TIMEOUT_MS:
                set_state(mem, STOP, "BORDER TIMEOUT")
                continue

            motors.arcade(config.BORDER_THROTTLE, 0.0)

            if sumw <= 2:
                mem.border_ok += 1
                if mem.border_ok >= config.BORDER_EXIT_N:
                    mem.border_ok = 0
                    mem.acquire_ok = 0
                    set_state(mem, ACQUIRE, "border cleared, acquiring clean line")
            else:
                mem.border_ok = 0
            continue
        """
 
        # ---------------- BORDER_PUSH ----------------
        if mem.state == BORDER_PUSH:
            if ticks_diff(t_now, mem.state_t0) >= config.BORDER_TIMEOUT_MS:
                set_state(mem, STOP, "BORDER TIMEOUT")
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
                    set_state(mem, ACQUIRE, "border cleared, acquiring clean line")
            else:
                mem.border_ok = 0
            continue
        # ---------------- ACQUIRE ----------------
        if mem.state == ACQUIRE:
            if ticks_diff(t_now, mem.state_t0) >= config.ACQUIRE_TIMEOUT_MS:
                set_state(mem, STOP, "ACQUIRE TIMEOUT")
                continue

            if sumw >= 3:
                mem.border_ok = 0
                set_state(mem, BORDER_PUSH, "wide white returned during acquire")
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
                    set_state(mem, GATE_CLEAR, "acquired good line, gate clear")
            else:
                mem.acquire_ok = 0
            continue

        # ---------------- GATE_CLEAR ----------------
        if mem.state == GATE_CLEAR:
            motors.arcade(config.BORDER_THROTTLE, 0.0)
            if ticks_diff(t_now, mem.state_t0) >= config.GATE_CLEAR_MS:
                mem.events_armed = False
                set_state(mem, ARM_WAIT, "gate cleared, arm wait")
            continue

        # ---------------- ARM_WAIT ----------------
        if mem.state == ARM_WAIT:
            motors.arcade(config.BORDER_THROTTLE, 0.0)
            if ticks_diff(t_now, mem.state_t0) >= config.ARM_DELAY_MS:
                mem.events_armed = True
                mem.last_event_t = t_now
                mem.good_rearm = 0
                mem.event_in_count = 0
                print("ARMED. step=0 act=", expected_action(mission, mem.step))
                set_state(mem, FOLLOW, "begin follow")
            continue

        # ---------------- DO_STRAIGHT ----------------
        if mem.state == DO_STRAIGHT:
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
                    set_state(mem, FINAL_FOLLOW, "final straight complete -> final follow")
                else:
                    set_state(mem, FOLLOW, "straight complete")
                continue

            if elapsed >= config.STRAIGHT_TIMEOUT_MS:
                mem.last_event_t = t_now
                mem.good_rearm = 0
                set_state(mem, FOLLOW, "straight timeout -> follow")
            continue

        # ---------------- TURN_APPROACH ----------------
        if mem.state == TURN_APPROACH:
            motors.arcade(config.STRAIGHT_THROTTLE, 0.0)
            if ticks_diff(t_now, mem.state_t0) >= config.TURN_APPROACH_MS:
                set_state(mem, TURN_SPIN, "approach done -> spin")
            continue

        # ---------------- TURN_SPIN ----------------
        if mem.state == TURN_SPIN:
            elapsed = ticks_diff(t_now, mem.state_t0)
            if elapsed >= config.TURN_TIMEOUT_MS:
                set_state(mem, STOP, "TURN TIMEOUT step={}".format(mem.step))
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
                set_state(mem, TURN_ALIGN, "sighted new branch, aligning")
            continue

        # ---------------- TURN_ALIGN ----------------
        if mem.state == TURN_ALIGN:
            if ticks_diff(t_now, mem.state_t0) >= config.ALIGN_TIMEOUT_MS:
                set_state(mem, STOP, "ALIGN TIMEOUT step={}".format(mem.step))
                continue

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

                    if mem.finish_mode and mem.step >= len(mission):
                        set_state(mem, FINAL_FOLLOW, "final turn complete -> final follow")
                    else:
                        set_state(mem, FOLLOW, "turn complete")
            else:
                mem.acquire_ok = 0
            continue

        # ---------------- FOLLOW ----------------
        if mem.state == FOLLOW:
            if mem.step >= len(mission):
                set_state(mem, FINAL_FOLLOW, "mission complete -> final follow")
                continue

            # update last_search_dir memory
            if err is not None:
                mem.last_err = err
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

            if can_detect:
                if event_cond:
                    mem.event_in_count += 1
                    if mem.event_in_count >= config.EVENT_ENTER_N:
                        mem.event_in_count = 0
                        mem.last_event_t = t_now
                        mem.good_rearm = 0

                        act = expected_action(mission, mem.step)
                        if act is None:
                            set_state(mem, FINAL_FOLLOW, "mission complete -> final follow")
                            continue

                        # If we're about to execute the last scripted action, enter finish mode.
                        if mem.step == (len(mission) - 1):
                            mem.finish_mode = True

                        mem.last_event_was_intersection = bool(eff_inter)

                        print(
                            "EVENT step=",
                            mem.step,
                            "act=",
                            act,
                            "inter=",
                            int(eff_inter),
                            "corner=",
                            int(corner_cond),
                            "w=",
                            b4(white),
                        )

                        if act == "straight":
                            mem.straight_out = 0
                            set_state(mem, DO_STRAIGHT, "event->straight")
                            continue

                        if act == "left":
                            mem.dir_turn = -1
                            if mem.last_event_was_intersection:
                                set_state(mem, TURN_APPROACH, "event->left (approach)")
                            else:
                                set_state(mem, TURN_SPIN, "event->left (corner)")
                            continue

                        if act == "right":
                            mem.dir_turn = +1
                            if mem.last_event_was_intersection:
                                set_state(mem, TURN_APPROACH, "event->right (approach)")
                            else:
                                set_state(mem, TURN_SPIN, "event->right (corner)")
                            continue

                        set_state(mem, STOP, "bad action {}".format(act))
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

        # ---------------- FINAL_FOLLOW ----------------
        if mem.state == FINAL_FOLLOW:
            set_state(mem, FINAL_FORWARD, "final follow done -> final forward")

        # ---------------- FINAL_FORWARD ----------------
        if mem.state == FINAL_FORWARD:
            motors.arcade(config.FINAL_FORWARD_THROTTLE, 0.0)
            if ticks_diff(t_now, mem.state_t0) >= config.FINAL_FORWARD_MS:
                set_state(mem, STOP, "final forward complete")
            continue
    

main()
