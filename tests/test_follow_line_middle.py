from utime import ticks_ms, ticks_diff, sleep_ms

import config
from hw.line import LineSensors
from hw.motors import DCMotor, MotorPair


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def bits4(x):
    x &= 0xF
    s = ""
    for k in (3, 2, 1, 0):
        s += "1" if (x & (1 << k)) else "0"
    return s


def middle_error(black, last_err):
    """
    black = [L, ML, MR, R] where 1 means black detected.
    Use ONLY ML and MR to compute error.

    Returns:
      err in {-1.0, 0.0, +1.0} or None if ML=MR=0 (lost).
    """
    mL = black[1]
    mR = black[2]

    if mL == 1 and mR == 0:
        return -1.0
    if mL == 0 and mR == 1:
        return +1.0
    if mL == 1 and mR == 1:
        return 0.0

    # mL==0 and mR==0
    return None


def main():
    sensors = LineSensors(config.LINE_PINS, invert=getattr(config, "LINE_INVERT", False))

    left = DCMotor(config.MOTOR_L_DIR, config.MOTOR_L_PWM, pwm_freq_hz=1000,
                   invert=getattr(config, "MOTOR_L_INVERT", True))
    right = DCMotor(config.MOTOR_R_DIR, config.MOTOR_R_PWM, pwm_freq_hz=1000,
                    invert=getattr(config, "MOTOR_R_INVERT", True))
    motors = MotorPair(left, right)

    loop_hz = getattr(config, "LOOP_HZ", 50)
    period_ms = int(1000 / loop_hz)
    dt_s = period_ms / 1000.0

    # PD + limits
    base = getattr(config, "BASE_THROTTLE", 0.20)
    kp = getattr(config, "KP", 0.35)      # middle-only usually wants higher KP
    kd = getattr(config, "KD", 0.04)
    max_steer = getattr(config, "MAX_STEER", 0.60)

    # Optional: slow down when steering hard (prevents oscillation)
    min_throttle = getattr(config, "MIN_THROTTLE", 0.10)
    slow_k = getattr(config, "SLOW_K", 0.10)  # throttle reduction per |steer|

    # Lost behaviour (still middle-only)
    lost_n = getattr(config, "LINE_LOST_N", 2)
    search_steer = getattr(config, "SEARCH_STEER", 0.45)
    search_throttle = getattr(config, "SEARCH_THROTTLE", 0.0)  # spin in place by default

    print_cooldown = getattr(config, "PRINT_COOLDOWN_MS", 250)
    last_print = 0

    last_err = 0.0
    lost_count = 0

    t_last = ticks_ms()

    while True:
        # fixed-rate loop
        t_now = ticks_ms()
        dt = ticks_diff(t_now, t_last)
        if dt < period_ms:
            sleep_ms(period_ms - dt)
            t_now = ticks_ms()
        t_last = t_now

        black = sensors.read_black()
        bits = sensors.read_bits()

        err = middle_error(black, last_err)

        # -------- Lost (ML=MR=0) --------
        if err is None:
            lost_count += 1

            if ticks_diff(t_now, last_print) > print_cooldown:
                print("MIDDLE LOST",
                      "bits=", bits4(bits),
                      "black=", black,
                      "last_err=", last_err)
                last_print = t_now

            if lost_count < lost_n:
                motors.arcade(0.0, 0.0)
            else:
                # turn in direction of last known error
                steer = search_steer if last_err >= 0 else -search_steer
                motors.arcade(search_throttle, steer)
            continue

        lost_count = 0

        # -------- PD control --------
        derr = (err - last_err) / dt_s
        steer = kp * err + kd * derr
        steer = clamp(steer, -max_steer, +max_steer)

        throttle = base - slow_k * abs(steer)
        throttle = clamp(throttle, min_throttle, base)

        motors.arcade(throttle, steer)
        last_err = err


main()