import config
from control.ulilities import clamp

def middle_error_white_line(black):
    mL = black[1]
    mR = black[2]
    if mL == 0 and mR == 0:
        return 0.0
    if mL == 1 and mR == 0:
        return +1.0
    if mL == 0 and mR == 1:
        return -1.0
    return None

def pd_follow(err, last_err, dt_s):
    derr = (err - last_err) / dt_s
    steer = config.KP * err + config.KD * derr
    steer = clamp(steer, -config.MAX_STEER, +config.MAX_STEER)

    throttle = config.BASE_THROTTLE - config.SLOW_K * abs(steer)
    throttle = clamp(throttle, config.MIN_THROTTLE, config.BASE_THROTTLE)
    return throttle, steer
