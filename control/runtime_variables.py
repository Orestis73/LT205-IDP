from utime import ticks_ms, ticks_diff, sleep_ms

class Mem:
    """All mutable runtime variables in one place."""

    def __init__(self):
        self.state = 0
        self.state_t0 = ticks_ms()

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


class State:
    START_BOX = 0
    BORDER_PUSH = 1
    ACQUIRE = 2
    GATE_CLEAR = 3
    FOLLOW = 4
    DO_STRAIGHT = 5
    TURN_APPROACH = 6
    TURN_SPIN = 7
    TURN_ALIGN = 8
    FINAL_FOLLOW = 9
    FINAL_FORWARD = 10
    STOP = 11