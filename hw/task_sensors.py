class TaskSensors:
    """
    Hardware abstraction for all non-line sensors / actuators.

    Safe to run without real hardware.
    Replace the test returns with the real logic later.
    """

    def __init__(self):
        # One reel per stack, 1-based slot index.
        self.test_reel_slot = {
            "od": None,
            "ou": None,
            "pu": None,
            "pd": None,
        }

        self.test_reel_colour = {
            "od": None,
            "ou": None,
            "pu": None,
            "pd": None,
        }

    # ---------------- start / stop ----------------
    def start_pressed(self):
        return True

    def stop_pressed(self):
        return False

    # ---------------- scanning ----------------
    def branch_has_reel(self, stack_name, slot_index, side):
        """
        Replace with side distance sensor logic.
        """
        expected = self.test_reel_slot.get(stack_name)
        return expected is not None and expected == slot_index

    # ---------------- pickup ----------------
    def pickup_target_reached(self, stack_name):
        """
        Replace with front ultrasonic logic.
        """
        return False

    def close_gripper(self):
        return None

    def reel_secured(self):
        return True

    def measure_reel_resistance(self):
        """
        Return measured resistance in ohms, or None until implemented.
        """
        return None

    def classify_resistance(self, resistance_ohms):
        """
        Convert resistance to:
            'blue', 'green', 'yellow', 'red'
        """
        return None

    def identify_picked_reel(self, stack_name):
        resistance = self.measure_reel_resistance()
        colour = self.classify_resistance(resistance)
        if colour is not None:
            return colour
        return self.test_reel_colour.get(stack_name)

    # ---------------- placement ----------------
    def drop_target_reached(self, colour_name):
        return False

    def open_gripper(self):
        return None

    def reel_released(self):
        return True

    # ---------------- indicator LEDs ----------------
    def show_detected_colour(self, colour_name):
        return None