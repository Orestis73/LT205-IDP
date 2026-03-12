from collections import deque


HEAD_N = 0
HEAD_E = 1
HEAD_S = 2
HEAD_W = 3


def heading_name(h):
    return {
        HEAD_N: "N",
        HEAD_E: "E",
        HEAD_S: "S",
        HEAD_W: "W",
    }[h]


def turn_name(turn_dir):
    return "R" if turn_dir > 0 else "L"


def rotate_90(heading, turn_dir):
    """
    turn_dir:
        -1 -> left
        +1 -> right
    """
    return (heading + turn_dir) % 4


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
    """
    High-level planner for the fixed IDP map.

    IMPORTANT MODEL:
    - Scan loop is explicit and fixed.
    - Reel scan points only exist at actual reel branches:
        pd = 4..9
        pu = 15..20
        ou = 23..28
        od = 32..37
    - Grab is a local detour:
        enter branch -> grab -> reverse back to the SAME node
      and STOP there, still facing along the branch axis.
    - Delivery is planned from that true post-grab pose.
    - Delivery goes to A / B / C / D, not directly to 38 / 39 / 2 / 3.
    - Place is also a local bay detour:
        enter bay -> drop -> fixed reverse -> local exit spin -> stop at A/B/C/D
      and return-home is planned from that post-place pose.
    """

    def __init__(self):
        self.scan_order = ["pd", "pu", "ou", "od"]

        self.stack_info = {
            "pd": {"slots": 6, "found_slot": None, "colour": None, "resolved": False},
            "pu": {"slots": 6, "found_slot": None, "colour": None, "resolved": False},
            "ou": {"slots": 6, "found_slot": None, "colour": None, "resolved": False},
            "od": {"slots": 6, "found_slot": None, "colour": None, "resolved": False},
        }

        self.mode = "scan"
        self.current_mission = None

        self.active_stack = None
        self.current_stack_turn = None

        # True pose after local grab macro returns to the pickup node
        self.rejoin_node = None
        self.rejoin_heading = None

        # Current delivery / place context
        self.current_colour = None
        self.place_approach_node = None

        # True pose after local place macro finishes and hands back to navigation
        self.post_place_node = None
        self.post_place_heading = None

        # Route graph
        self.graph = {
            1: [2, 39],
            2: [1, 3, "C"],
            3: [2, 4, "D"],
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
            38: [37, 39, "A"],
            39: [38, 1, "B"],
            "A": [38],
            "B": [39],
            "C": [2],
            "D": [3],
        }

        # Coordinates only used to infer heading between connected nodes
        self.xy = {
            1: (0, 0),
            2: (3, 0),
            3: (4, 0),

            4: (4, 1),
            5: (4, 2),
            6: (4, 3),
            7: (4, 4),
            8: (4, 5),
            9: (4, 6),
            10: (4, 7),
            11: (4, 8),

            12: (0, 8),
            13: (0, 2),

            14: (1, 2),
            15: (1, 3),
            16: (1, 4),
            17: (1, 5),
            18: (1, 6),
            19: (1, 7),
            20: (1, 8),
            21: (1, 9),

            22: (-1, 2),
            23: (-1, 3),
            24: (-1, 4),
            25: (-1, 5),
            26: (-1, 6),
            27: (-1, 7),
            28: (-1, 8),
            29: (-1, 9),

            30: (-4, 8),
            31: (-4, 7),
            32: (-4, 6),
            33: (-4, 5),
            34: (-4, 4),
            35: (-4, 3),
            36: (-4, 2),
            37: (-4, 1),
            38: (-4, 0),
            39: (-3, 0),

            "A": (-4, -1),
            "B": (-3, -1),
            "C": (3, -1),
            "D": (4, -1),
        }

        # Explicit perimeter scan loop
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

        # Scan points only on REAL reel branches.
        # scan_heading = direction of travel for which the scan is valid.
        # This avoids duplicate scanning on the reverse pass.
        self.scan_points = {
            # pd: right outer wall, moving north, branch is inward/left
            4: {"stack": "pd", "slot": 1, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            5: {"stack": "pd", "slot": 2, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            6: {"stack": "pd", "slot": 3, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            7: {"stack": "pd", "slot": 4, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            8: {"stack": "pd", "slot": 5, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            9: {"stack": "pd", "slot": 6, "turn": -1, "side": "left", "scan_heading": HEAD_N},

            # pu: branches 15..20 are to the RIGHT/east of the 14->21 vertical
            15: {"stack": "pu", "slot": 1, "turn": +1, "side": "right", "scan_heading": HEAD_N},
            16: {"stack": "pu", "slot": 2, "turn": +1, "side": "right", "scan_heading": HEAD_N},
            17: {"stack": "pu", "slot": 3, "turn": +1, "side": "right", "scan_heading": HEAD_N},
            18: {"stack": "pu", "slot": 4, "turn": +1, "side": "right", "scan_heading": HEAD_N},
            19: {"stack": "pu", "slot": 5, "turn": +1, "side": "right", "scan_heading": HEAD_N},
            20: {"stack": "pu", "slot": 6, "turn": +1, "side": "right", "scan_heading": HEAD_N},

            # ou: branches 23..28 are to the LEFT/west of the 22->29 vertical
            23: {"stack": "ou", "slot": 1, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            24: {"stack": "ou", "slot": 2, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            25: {"stack": "ou", "slot": 3, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            26: {"stack": "ou", "slot": 4, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            27: {"stack": "ou", "slot": 5, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            28: {"stack": "ou", "slot": 6, "turn": -1, "side": "left", "scan_heading": HEAD_N},

            # od: left outer wall, moving south, branch is inward/left
            32: {"stack": "od", "slot": 1, "turn": -1, "side": "left", "scan_heading": HEAD_S},
            33: {"stack": "od", "slot": 2, "turn": -1, "side": "left", "scan_heading": HEAD_S},
            34: {"stack": "od", "slot": 3, "turn": -1, "side": "left", "scan_heading": HEAD_S},
            35: {"stack": "od", "slot": 4, "turn": -1, "side": "left", "scan_heading": HEAD_S},
            36: {"stack": "od", "slot": 5, "turn": -1, "side": "left", "scan_heading": HEAD_S},
            37: {"stack": "od", "slot": 6, "turn": -1, "side": "left", "scan_heading": HEAD_S},
        }

        # Global delivery target = bay-approach node only
        self.delivery_targets = {
            "blue": {"approach": "A"},
            "green": {"approach": "B"},
            "yellow": {"approach": "C"},
            "red": {"approach": "D"},
        }

        # Local place macro behaviour after reaching A / B / C / D.
        # I made exit_action = 180 so return-home starts from a corridor-facing pose.
        self.place_cfg = {
            "blue": {
                "approach": "A",
                "enter_action": "straight",
                "exit_action": "180",
                "exit_spin_dir": +1,
                "exit_heading": HEAD_N,
            },
            "green": {
                "approach": "B",
                "enter_action": "straight",
                "exit_action": "180",
                "exit_spin_dir": +1,
                "exit_heading": HEAD_N,
            },
            "yellow": {
                "approach": "C",
                "enter_action": "straight",
                "exit_action": "180",
                "exit_spin_dir": +1,
                "exit_heading": HEAD_N,
            },
            "red": {
                "approach": "D",
                "enter_action": "straight",
                "exit_action": "180",
                "exit_spin_dir": +1,
                "exit_heading": HEAD_N,
            },
        }

    def all_resolved(self):
        for stack_name in self.scan_order:
            if not self.stack_info[stack_name]["resolved"]:
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

    def _default_180_dir(self, node, heading_in, heading_out):
        """
        Physical spin direction for 180s.

        User requirement:
        - top U-turn on pu (node 21) should be leftward
        - top U-turn on ou (node 29) should be rightward
        """
        if node == 21:
            return -1
        if node == 29:
            return +1
        return +1

    def _build_mission_from_node_path(self, node_path, start_heading, scan_enabled=False):
        mission = []
        current_heading = start_heading

        for i in range(len(node_path) - 1):
            node = node_path[i]
            nxt = node_path[i + 1]
            next_heading = self._heading_between(node, nxt)
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
                info = self.scan_points[node]
                if current_heading == info["scan_heading"]:
                    scan_meta = dict(info)
                    del scan_meta["scan_heading"]
                    step["scan"] = scan_meta

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

    def register_reel_found(self, stack_name, slot_index, turn_dir, node, corridor_heading):
        """
        Called when a reel is detected during scan.

        IMPORTANT:
        grab_movement() turns INTO the branch and later reverses back to the same node,
        but does NOT turn back onto the corridor.

        Therefore the post-grab heading is the BRANCH heading, i.e. corridor heading
        rotated by the branch entry turn.
        """
        self.active_stack = stack_name
        self.current_stack_turn = turn_dir
        self.stack_info[stack_name]["found_slot"] = slot_index

        self.rejoin_node = node
        self.rejoin_heading = rotate_90(corridor_heading, turn_dir)

        self.mode = "pick"

    def build_delivery_mission(self, colour_name):
        if self.active_stack is None:
            raise ValueError("No active stack")
        if colour_name not in self.delivery_targets:
            raise ValueError("Invalid colour {}".format(colour_name))
        if self.rejoin_node is None or self.rejoin_heading is None:
            raise ValueError("Rejoin pose not set")

        self.stack_info[self.active_stack]["colour"] = colour_name
        self.current_colour = colour_name

        target = self.delivery_targets[colour_name]
        approach = target["approach"]

        self.place_approach_node = approach

        node_path = self._shortest_path(self.rejoin_node, approach)
        self.current_mission = self._build_mission_from_node_path(
            node_path,
            start_heading=self.rejoin_heading,
            scan_enabled=False,
        )
        self.mode = "to_place_approach"
        return self.current_mission

    def set_post_place_pose(self, node, heading):
        self.post_place_node = node
        self.post_place_heading = heading

    def set_post_place_pose_from_current_colour(self):
        if self.current_colour is None:
            raise ValueError("No current colour set")

        cfg = self.place_cfg[self.current_colour]
        self.set_post_place_pose(cfg["approach"], cfg["exit_heading"])

    def build_return_home_mission(self):
        if self.post_place_node is None or self.post_place_heading is None:
            raise ValueError("Post-place pose not set")

        path = self._shortest_path(self.post_place_node, 1)
        self.current_mission = self._build_mission_from_node_path(
            path,
            start_heading=self.post_place_heading,
            scan_enabled=False,
        )
        self.mode = "return_home"
        return self.current_mission

    def complete_delivery_cycle(self):
        self.stack_info[self.active_stack]["resolved"] = True

        self.active_stack = None
        self.current_stack_turn = None

        self.rejoin_node = None
        self.rejoin_heading = None

        self.current_colour = None
        self.place_approach_node = None

        self.post_place_node = None
        self.post_place_heading = None

        self.mode = "scan"

    def get_step(self, mission, step_index):
        if step_index < 0 or step_index >= len(mission["steps"]):
            return None
        return mission["steps"][step_index]