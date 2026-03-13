
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

    Corrected mission policy:
    - There are only 4 real reels total.
    - A stack can contain multiple reels.
    - Scan progress is persistent: do NOT restart from node 1 after each delivery.
    - After a delivery, resume scanning from the next unresolved scan point.
    - Return to node 1 only once, after all 4 reels have been delivered.

    Corrected local/global split:
    - Grab is a local detour:
        turn into branch -> grab -> reverse back to SAME node
      and stop there still facing along the branch axis.
    - Delivery is global navigation to A/B/C/D only.
    - Place is a local detour from A/B/C/D.
    - After place, planner resumes scan from the post-place pose.
    """

    def __init__(self, expected_total_reels=4):
        self.expected_total_reels = expected_total_reels

        self.scan_order = ["pd", "pu", "ou", "od"]

        # Per-stack bookkeeping
        self.stack_info = {
            "pd": {
                "slots": 6,
                "checked_slots": set(),
                "detected_slots": set(),
                "delivered_slots": set(),
                "colours_by_slot": {},
            },
            "pu": {
                "slots": 6,
                "checked_slots": set(),
                "detected_slots": set(),
                "delivered_slots": set(),
                "colours_by_slot": {},
            },
            "ou": {
                "slots": 6,
                "checked_slots": set(),
                "detected_slots": set(),
                "delivered_slots": set(),
                "colours_by_slot": {},
            },
            "od": {
                "slots": 6,
                "checked_slots": set(),
                "detected_slots": set(),
                "delivered_slots": set(),
                "colours_by_slot": {},
            },
        }

        self.mode = "scan"
        self.current_mission = None

        # Current active pickup/delivery context
        self.active_stack = None
        self.active_slot = None
        self.current_stack_turn = None

        # True pose after local grab macro returns to the pickup node
        self.rejoin_node = None
        self.rejoin_heading = None

        # Current place context
        self.current_colour = None
        self.place_approach_node = None

        # True pose after local place macro finishes
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

        # Manhattan coordinates
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

        # Explicit full perimeter scan loop
        self.scan_loop_nodes = [
            1,
            2, 3,
            4, 5, 6, 7, 8, 9, 10, 11,
            12, 13,
            14, 15, 16, 17, 18, 19, 20,
            19, 18, 17, 16, 15, 14,
            13,
            22, 23, 24, 25, 26, 27, 28,
            27, 26, 25, 24, 23, 22,
            13, 12,
            30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
            1,
        ]

        # Real scan points only
        self.scan_points = {
            # pd: moving north, branch inward/left
            4: {"stack": "pd", "slot": 1, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            5: {"stack": "pd", "slot": 2, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            6: {"stack": "pd", "slot": 3, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            7: {"stack": "pd", "slot": 4, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            8: {"stack": "pd", "slot": 5, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            9: {"stack": "pd", "slot": 6, "turn": -1, "side": "left", "scan_heading": HEAD_N},

            # pu: moving north, branch inward/right
            15: {"stack": "pu", "slot": 1, "turn": +1, "side": "right", "scan_heading": HEAD_N},
            16: {"stack": "pu", "slot": 2, "turn": +1, "side": "right", "scan_heading": HEAD_N},
            17: {"stack": "pu", "slot": 3, "turn": +1, "side": "right", "scan_heading": HEAD_N},
            18: {"stack": "pu", "slot": 4, "turn": +1, "side": "right", "scan_heading": HEAD_N},
            19: {"stack": "pu", "slot": 5, "turn": +1, "side": "right", "scan_heading": HEAD_N},
            20: {"stack": "pu", "slot": 6, "turn": +1, "side": "right", "scan_heading": HEAD_N},

            # ou: moving north, branch inward/left
            23: {"stack": "ou", "slot": 1, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            24: {"stack": "ou", "slot": 2, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            25: {"stack": "ou", "slot": 3, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            26: {"stack": "ou", "slot": 4, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            27: {"stack": "ou", "slot": 5, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            28: {"stack": "ou", "slot": 6, "turn": -1, "side": "left", "scan_heading": HEAD_N},

            # od: moving south, branch inward/left
            32: {"stack": "od", "slot": 1, "turn": -1, "side": "left", "scan_heading": HEAD_S},
            33: {"stack": "od", "slot": 2, "turn": -1, "side": "left", "scan_heading": HEAD_S},
            34: {"stack": "od", "slot": 3, "turn": -1, "side": "left", "scan_heading": HEAD_S},
            35: {"stack": "od", "slot": 4, "turn": -1, "side": "left", "scan_heading": HEAD_S},
            36: {"stack": "od", "slot": 5, "turn": -1, "side": "left", "scan_heading": HEAD_S},
            37: {"stack": "od", "slot": 6, "turn": -1, "side": "left", "scan_heading": HEAD_S},
        }

        self.delivery_targets = {
            "blue": {"approach": "A"},
            "green": {"approach": "B"},
            "yellow": {"approach": "C"},
            "red": {"approach": "D"},
        }

        # Local place behaviour after reaching A/B/C/D
        self.place_cfg = {
            "blue": {
                "approach": "A",
                "enter_action": "straight",
                "exit_action": "none",
                "exit_spin_dir": +1,
                "exit_heading": HEAD_S,
            },
            "green": {
                "approach": "B",
                "enter_action": "straight",
                "exit_action": "none",
                "exit_spin_dir": +1,
                "exit_heading": HEAD_S,
            },
            "yellow": {
                "approach": "C",
                "enter_action": "straight",
                "exit_action": "none",
                "exit_spin_dir": +1,
                "exit_heading": HEAD_S,
            },
            "red": {
                "approach": "D",
                "enter_action": "straight",
                "exit_action": "none",
                "exit_spin_dir": +1,
                "exit_heading": HEAD_S,
            },
        }

        # Full scan template from the canonical start pose
        self.scan_template = self._build_mission_from_node_path(
            self.scan_loop_nodes,
            start_heading=HEAD_N,
            scan_enabled=True,
        )

        # Ordered real scan sequence
        self.scan_sequence = []
        self.scan_step_index_by_slot = {}
        self.scan_node_by_slot = {}

        for idx, step in enumerate(self.scan_template["steps"]):
            scan = step.get("scan")
            if scan is not None:
                key = (scan["stack"], scan["slot"])
                self.scan_sequence.append(key)
                self.scan_step_index_by_slot[key] = idx
                self.scan_node_by_slot[key] = step["node"]

        self.scan_cursor = 0

    # ------------------------------------------------------------------
    # Basic bookkeeping
    # ------------------------------------------------------------------
    def _clone_steps(self, steps):
        cloned = []

        for step in steps:
            new_step = {}
            for k, v in step.items():
                if k == "scan" and v is not None:
                    new_step[k] = dict(v)
                else:
                    new_step[k] = v
            cloned.append(new_step)

        return cloned


    def total_delivered(self):
        total = 0
        for stack_name in self.scan_order:
            total += len(self.stack_info[stack_name]["delivered_slots"])
        return total

    def total_slots(self):
        total = 0
        for stack_name in self.scan_order:
            total += self.stack_info[stack_name]["slots"]
        return total

    def reels_remaining_to_find(self):
        remaining = self.expected_total_reels - self.total_delivered()
        return remaining if remaining > 0 else 0

    def all_resolved(self):
        return self.total_delivered() >= self.expected_total_reels

    def slot_checked(self, stack_name, slot_index):
        return slot_index in self.stack_info[stack_name]["checked_slots"]

    def slot_is_delivered(self, stack_name, slot_index):
        return slot_index in self.stack_info[stack_name]["delivered_slots"]

    def slot_needs_scan(self, stack_name, slot_index):
        return not self.slot_checked(stack_name, slot_index)

    def stack_needs_scan(self, stack_name):
        info = self.stack_info[stack_name]
        return len(info["checked_slots"]) < info["slots"]

    def mark_slot_checked(self, stack_name, slot_index):
        self.stack_info[stack_name]["checked_slots"].add(slot_index)
        self._advance_scan_cursor_past_checked()

    def _advance_scan_cursor_past_checked(self):
        while self.scan_cursor < len(self.scan_sequence):
            stack_name, slot_index = self.scan_sequence[self.scan_cursor]
            if self.slot_checked(stack_name, slot_index):
                self.scan_cursor += 1
            else:
                break

    def next_unchecked_slot(self):
        self._advance_scan_cursor_past_checked()
        if self.scan_cursor >= len(self.scan_sequence):
            return None
        return self.scan_sequence[self.scan_cursor]

    # ------------------------------------------------------------------
    # Geometry / planning helpers
    # ------------------------------------------------------------------

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
        q = [start]
        q_head = 0
        prev = {start: None}

        while q_head < len(q):
            n = q[q_head]
            q_head += 1

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

    def _pose_shortest_path(self, start_node, start_heading, goal_node, goal_heading):
        """
        BFS in pose-space:
            state = (node, heading)

        We use this so resume-scan can rejoin the perimeter with the
        correct heading for the next scan point, instead of just reaching
        the right node from the wrong direction.
        """
        start_state = (start_node, start_heading)
        goal_state = (goal_node, goal_heading)

        if start_state == goal_state:
            return [start_node]

        q = [start_state]
        q_head = 0
        prev = {start_state: None}

        found = False

        while q_head < len(q):
            node, heading = q[q_head]
            q_head += 1

            for nxt in self.graph[node]:
                new_heading = self._heading_between(node, nxt)
                nxt_state = (nxt, new_heading)

                if nxt_state not in prev:
                    prev[nxt_state] = (node, heading)

                    if nxt_state == goal_state:
                        found = True
                        break

                    q.append(nxt_state)

            if found:
                break

        if goal_state not in prev:
            raise ValueError(
                "No pose-path from ({}, {}) to ({}, {})".format(
                    start_node,
                    heading_name(start_heading),
                    goal_node,
                    heading_name(goal_heading),
                )
            )

        states = []
        s = goal_state
        while s is not None:
            states.append(s)
            s = prev[s]
        states.reverse()

        return [node for node, _heading in states]

    def _default_180_dir(self, node, heading_in, heading_out):
        # Required special cases
        if node == 20:
            return +1
        if node == 28:
            return -1
        return +1

    def _build_mission_from_node_path(self, node_path, start_heading, scan_enabled=False):
        if len(node_path) == 0:
            raise ValueError("Empty node_path")

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

    def _mission_suffix_from_scan_step_index(self, start_step_idx):
        steps = self._clone_steps(self.scan_template["steps"][start_step_idx:])
        return {
            "steps": steps,
            "end_node": self.scan_template["end_node"],
            "end_heading": self.scan_template["end_heading"],
        }
    
    def _concat_missions(self, first, second):
        if len(first["steps"]) == 0:
            return second
        if len(second["steps"]) == 0:
            return first

        return {
            "steps": first["steps"] + second["steps"],
            "end_node": second["end_node"],
            "end_heading": second["end_heading"],
        }

    def _build_scan_campaign_from_pose(self, start_node, start_heading):
        """
        Build a mission that:
        1. Reaches the next unresolved scan point in the CORRECT scan heading.
        2. Then continues along the canonical scan template from there onward.
        """
        if self.all_resolved():
            raise ValueError("All reels already delivered")

        next_slot = self.next_unchecked_slot()
        if next_slot is None:
            raise ValueError("No unresolved scan slots remain")

        step_idx = self.scan_step_index_by_slot[next_slot]
        target_step = self.scan_template["steps"][step_idx]
        target_node = target_step["node"]
        target_heading = target_step["heading_in"]

        pose_path = self._pose_shortest_path(
            start_node=start_node,
            start_heading=start_heading,
            goal_node=target_node,
            goal_heading=target_heading,
        )

        prefix = self._build_mission_from_node_path(
            pose_path,
            start_heading=start_heading,
            scan_enabled=False,
        )

        suffix = self._mission_suffix_from_scan_step_index(step_idx)
        mission = self._concat_missions(prefix, suffix)

        self.current_mission = mission
        return mission

    # ------------------------------------------------------------------
    # Public planner API
    # ------------------------------------------------------------------

    def build_initial_scan_mission(self):
        self.mode = "scan"
        return self._build_scan_campaign_from_pose(1, HEAD_N)

    def build_resume_scan_mission(self):
        if self.post_place_node is None or self.post_place_heading is None:
            raise ValueError("Post-place pose not set")

        self.mode = "resume_scan"
        return self._build_scan_campaign_from_pose(
            self.post_place_node,
            self.post_place_heading,
        )

    # Backward-compatible legacy name
    def build_scan_loop_mission(self):
        return self.build_initial_scan_mission()

    def register_reel_found(self, stack_name, slot_index, turn_dir, node, corridor_heading):
        if self.slot_is_delivered(stack_name, slot_index):
            raise ValueError("Slot already delivered: {} slot {}".format(stack_name, slot_index))

        self.mark_slot_checked(stack_name, slot_index)
        self.stack_info[stack_name]["detected_slots"].add(slot_index)

        self.active_stack = stack_name
        self.active_slot = slot_index
        self.current_stack_turn = turn_dir

        # Grab is local and returns to same node facing along branch axis
        self.rejoin_node = node
        self.rejoin_heading = rotate_90(corridor_heading, turn_dir)

        self.mode = "pick"

    def build_delivery_mission(self, colour_name):
        if self.active_stack is None or self.active_slot is None:
            raise ValueError("No active reel selected")
        if colour_name not in self.delivery_targets:
            raise ValueError("Invalid colour {}".format(colour_name))
        if self.rejoin_node is None or self.rejoin_heading is None:
            raise ValueError("Rejoin pose not set")

        self.stack_info[self.active_stack]["colours_by_slot"][self.active_slot] = colour_name
        self.current_colour = colour_name

        approach = self.delivery_targets[colour_name]["approach"]
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
        if self.active_stack is None or self.active_slot is None:
            raise ValueError("No active reel to complete")

        self.stack_info[self.active_stack]["delivered_slots"].add(self.active_slot)

        self.active_stack = None
        self.active_slot = None
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