from utime import ticks_ms


class Mem:
    def __init__(self):
        self.state = 0
        self.state_t0 = ticks_ms()

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

        self.dbg_next = ticks_ms()
        self.dbg_count = 0

        self.step = 0

        self.grabbed_colour = None


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
    STOP = 9
    SPIN180_SPIN = 10
    SPIN180_ALIGN = 11
    GRAB = 12
    GRAB_FORWARD = 13
    GRAB_WAIT = 14
    GRAB_REVERSE = 15
    GRAB_TURN_BACK = 16   # unused, kept for compatibility
    PLACE = 17
    PLACE_FORWARD = 18
    PLACE_WAIT = 19
    PLACE_REVERSE = 20
    PLACE_TURN_BACK = 21  # unused, kept for compatibility