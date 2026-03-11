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
from control.movement import border_push_movement, spin180_movement, turning_movement, straight_movement, spin180_movement, grab_movement
from control.pd import middle_error_white_line, pd_follow
from state_machine import navigation

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

def get_new_mission(mem,nav):
    mem.step = 0
    mission = nav.route_executor()
    return mission


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
    
    while True:
        motors.arcade(60.0, 0.0)
        print ("moving")



main()


