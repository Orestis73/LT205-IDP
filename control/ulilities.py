from utime import ticks_ms

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def b4(v01):
    # list of 0/1 -> "0101"
    return "{}{}{}{}".format(v01[0], v01[1], v01[2], v01[3])

def set_state(mem, new_state, why=""):
    mem.state = new_state
    mem.state_t0 = ticks_ms()