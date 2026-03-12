from collections import deque


HEAD_N = 0
HEAD_E = 1
HEAD_S = 2
HEAD_W = 3


def _turn_action(h_in, h_out):
    delta = (h_out - h_in) % 4
    if delta == 0:
        return "straight"
    if delta == 1:
        return "right"
    if delta == 2:
        return "180"
    return "left"


class navigation:
    def __init__(self):
        self.scan_order = ["od", "ou", "pu", "pd"]
        self.stack_info = {
            "od": {"slots": 6, "found_slot": None, "colour": None, "resolved": False},
            "ou": {"slots": 6, "found_slot": None, "colour": None, "resolved": False},
            "pu": {"slots": 6, "found_slot": None, "colour": None, "resolved": False},
            "pd": {"slots": 7, "found_slot": None, "colour": None, "resolved": False},
        }

        self.active_stack = None
        self.pending_colour = None
        self.mode = "scan"
        self.current_mission = None
        self.return_entry_node = None
        self.current_stack_turn = None
        self.current_place_turn = None

        # REAL pose to resume from after local grab/place macros
        self.rejoin_node = None
        self.rejoin_heading = None

        self.graph = {
            1: [2, 39],
            2: [1, 3],
            3: [2, 4],
            4: [3, 5],
            5: [4, 6],
            6: [5, 7],
            7: [6, 8],
            8: [7, 9],
            9: [8, 10],
            10: [9, 11],
            11: [10, 12],
            12: [11, 13, 30],
            13: [12, 14, 22],
            14: [13, 15],
            15: [14, 16],
            16: [15, 17],
            17: [16, 18],
            18: [17, 19],
            19: [18, 20],
            20: [19, 21],
            21: [20],
            22: [13, 23],
            23: [22, 24],
            24: [23, 25],
            25: [24, 26],
            26: [25, 27],
            27: [26, 28],
            28: [27, 29],
            29: [28],
            30: [12, 31],
            31: [30, 32],
            32: [31, 33],
            33: [32, 34],
            34: [33, 35],
            35: [34, 36],
            36: [35, 37],
            37: [36, 38],
            38: [37, 39],
            39: [38, 1],
        }

        self.xy = {
            1: (0, 0), 2: (3, 0), 3: (4, 0),
            4: (4, 1), 5: (4, 2), 6: (4, 3), 7: (4, 4), 8: (4, 5), 9: (4, 6), 10: (4, 7), 11: (4, 8),
            12: (0, 8), 13: (0, 2),
            14: (1, 2), 15: (1, 3), 16: (1, 4), 17: (1, 5), 18: (1, 6), 19: (1, 7), 20: (1, 8), 21: (1, 9),
            22: (-1, 2), 23: (-1, 3), 24: (-1, 4), 25: (-1, 5), 26: (-1, 6), 27: (-1, 7), 28: (-1, 8), 29: (-1, 9),
            30: (-4, 8), 31: (-4, 7), 32: (-4, 6), 33: (-4, 5), 34: (-4, 4), 35: (-4, 3), 36: (-4, 2), 37: (-4, 1), 38: (-4, 0), 39: (-3, 0),
        }

        self.drop_entry = {
            "yellow": {"approach_prev": 1, "entry": 2, "turn": +1},
            "red":    {"approach_prev": 2, "entry": 3, "turn": +1},
            "green":  {"approach_prev": 1, "entry": 39, "turn": -1},
            "blue":   {"approach_prev": 39, "entry": 38, "turn": -1},
        }

        self.scan_loop_nodes = [
            1,
            2, 3,
            4, 5, 6, 7, 8, 9, 10, 11,
            12, 13,
            14, 15, 16, 17, 18, 19, 20, 21,
            20, 19, 18, 17, 16, 15, 14,
            13,
            22, 23, 24, 25, 26, 27, 28, 29,
            28, 27, 26, 25, 24, 23, 22,
            13, 12,
            30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
            1,
        ]

        self.scan_points = {
            4:  {"stack": "od", "slot": 1, "turn": -1, "side": "left"},
            5:  {"stack": "od", "slot": 2, "turn": -1, "side": "left"},
            6:  {"stack": "od", "slot": 3, "turn": -1, "side": "left"},
            7:  {"stack": "od", "slot": 4, "turn": -1, "side": "left"},
            8:  {"stack": "od", "slot": 5, "turn": -1, "side": "left"},
            9:  {"stack": "od", "slot": 6, "turn": -1, "side": "left"},

            15: {"stack": "ou", "slot": 1, "turn": +1, "side": "right"},
            16: {"stack": "ou", "slot": 2, "turn": +1, "side": "right"},
            17: {"stack": "ou", "slot": 3, "turn": +1, "side": "right"},
            18: {"stack": "ou", "slot": 4, "turn": +1, "side": "right"},
            19: {"stack": "ou", "slot": 5, "turn": +1, "side": "right"},
            20: {"stack": "ou", "slot": 6, "turn": +1, "side": "right"},

            23: {"stack": "pu", "slot": 1, "turn": -1, "side": "left"},
            24: {"stack": "pu", "slot": 2, "turn": -1, "side": "left"},
            25: {"stack": "pu", "slot": 3, "turn": -1, "side": "left"},
            26: {"stack": "pu", "slot": 4, "turn": -1, "side": "left"},
            27: {"stack": "pu", "slot": 5, "turn": -1, "side": "left"},
            28: {"stack": "pu", "slot": 6, "turn": -1, "side": "left"},

            31: {"stack": "pd", "slot": 1, "turn": -1, "side": "left"},
            32: {"stack": "pd", "slot": 2, "turn": -1, "side": "left"},
            33: {"stack": "pd", "slot": 3, "turn": -1, "side": "left"},
            34: {"stack": "pd", "slot": 4, "turn": -1, "side": "left"},
            35: {"stack": "pd", "slot": 5, "turn": -1, "side": "left"},
            36: {"stack": "pd", "slot": 6, "turn": -1, "side": "left"},
            37: {"stack": "pd", "slot": 7, "turn": -1, "side": "left"},
        }

    def all_resolved(self):
        for name in self.scan_order:
            if not self.stack_info[name]["resolved"]:
                return False
        return True

    def stack_needs_scan(self, stack_name):
        return not self.stack_info[stack_name]["resolved"]

    def _heading_between(self, a, b):
        xa, ya = self.xy[a]
        xb, yb = self.xy[b]
        if xb == xa and yb > ya:
            return HEAD_N
        if xb > xa and yb == ya:
            return HEAD_E
        if xb == xa and yb < ya:
            return HEAD_S
        if xb < xa and yb == ya:
            return HEAD_W
        raise ValueError("Non-Manhattan edge: {} -> {}".format(a, b))

    def _shortest_path(self, start, goal):
        q = deque([start])
        prev = {start: None}
        while q:
            n = q.popleft()
            if n == goal:
                break
            for nxt in self.graph[n]:
                if nxt not in prev:
                    prev[nxt] = n
                    q.append(nxt)
        if goal not in prev:
            raise ValueError("No path from {} to {}".format(start, goal))
        path = []
        n = goal
        while n is not None:
            path.append(n)
            n = prev[n]
        path.reverse()
        return path

    def _build_mission_from_node_path(self, node_path, start_heading, scan_enabled=False):
        mission = []
        current_heading = start_heading
        for i in range(len(node_path) - 1):
            node = node_path[i]
            nxt = node_path[i + 1]
            next_heading = self._heading_between(node, nxt)
            action = _turn_action(current_heading, next_heading)
            action = _turn_action(current_heading, next_heading)

            step = {
                "move": action,
                "node": node,
                "heading_in": current_heading,
                "heading_out": next_heading,
            }

            if action == "180":
                step["spin_dir"] = self._default_180_dir(node, current_heading, next_heading)

            if scan_enabled and node in self.scan_points:
                step["scan"] = self.scan_points[node]
            mission.append(step)
            current_heading = next_heading
        return {
            "steps": mission,
            "end_node": node_path[-1],
            "end_heading": current_heading,
        }

    def build_scan_loop_mission(self):
        self.mode = "scan"
        self.current_mission = self._build_mission_from_node_path(
            self.scan_loop_nodes,
            start_heading=HEAD_N,
            scan_enabled=True,
        )
        return self.current_mission

    def register_reel_found(self, stack_name, slot_index, turn_dir, rejoin_node, rejoin_heading):
        self.active_stack = stack_name
        self.current_stack_turn = turn_dir
        self.stack_info[stack_name]["found_slot"] = slot_index

        # Exact pose after the local grab macro returns to the main route
        self.rejoin_node = rejoin_node
        self.rejoin_heading = rejoin_heading

        self.mode = "pick"

    def _scan_node_for(self, stack_name, slot_index):
        for node, info in self.scan_points.items():
            if info["stack"] == stack_name and info["slot"] == slot_index:
                return node
        raise ValueError("Unknown stack/slot: {} {}".format(stack_name, slot_index))

    def _entry_heading_for_colour(self, colour_name):
        info = self.drop_entry[colour_name]
        return self._heading_between(info["approach_prev"], info["entry"])

    def build_delivery_mission(self, colour_name):
        if self.active_stack is None:
            raise ValueError("No active stack to deliver from")
        if colour_name not in self.drop_entry:
            raise ValueError("Invalid destination colour: {}".format(colour_name))
        if self.rejoin_node is None or self.rejoin_heading is None:
            raise ValueError("Rejoin pose not set")

        self.stack_info[self.active_stack]["colour"] = colour_name

        current_node = self.rejoin_node
        current_heading = self.rejoin_heading

        approach_prev = self.drop_entry[colour_name]["approach_prev"]
        entry = self.drop_entry[colour_name]["entry"]
        self.current_place_turn = self.drop_entry[colour_name]["turn"]

        path_to_prev = self._shortest_path(current_node, approach_prev)
        node_path = path_to_prev + [entry]

        self.return_entry_node = entry
        self.current_mission = self._build_mission_from_node_path(
            node_path,
            start_heading=current_heading,
            scan_enabled=False,
        )
        self.mode = "to_place"
        return self.current_mission

    def build_return_home_mission(self):
        if self.return_entry_node is None:
            raise ValueError("No return entry node stored")
        colour = self.stack_info[self.active_stack]["colour"]
        entry = self.return_entry_node
        heading = self._entry_heading_for_colour(colour)
        path = self._shortest_path(entry, 1)
        self.current_mission = self._build_mission_from_node_path(
            path,
            start_heading=heading,
            scan_enabled=False,
        )
        self.mode = "return_home"
        return self.current_mission

    def complete_delivery_cycle(self):
        self.stack_info[self.active_stack]["resolved"] = True
        self.active_stack = None
        self.pending_colour = None
        self.return_entry_node = None
        self.current_stack_turn = None
        self.current_place_turn = None
        self.rejoin_node = None
        self.rejoin_heading = None
        self.mode = "scan"

    def step_count(self, mission):
        return len(mission["steps"])

    def get_step(self, mission, step_index):
        if step_index < 0 or step_index >= len(mission["steps"]):
            return None
        return mission["steps"][step_index]
    
    def _default_180_dir(self, node, heading_in, heading_out):
        """
        Choose the physical spin direction for 180-degree turns.

        Return:
            -1 -> spin left
            +1 -> spin right
        """

        # Special cases you explicitly asked for:
        # top U-turn at PU should be rightward
        if node == 29:
            return +1

        # top U-turn at OU should be leftward
        if node == 21:
            return -1

        # sensible default for everything else
        return +1