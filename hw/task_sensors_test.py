from machine import Pin


class TaskSensors:
    """
    Hardware abstraction for all non-line sensors / actuators.

    This file is the bridge between the high-level state machine and the real
    hardware that is NOT part of line following.

    The rest of the robot code should call these methods and not care whether
    the result came from:
    - ultrasonic sensors
    - ToF sensors
    - analog resistance measurement
    - digital switches
    - servo commands
    - motor drivers
    - test values

    For now, each method is a stub.
    You must later replace the body of each method with the real hardware logic.
    """

    def __init__(self):
        # ------------------------------------------------------------------
        # TEST-ONLY FIELDS
        # ------------------------------------------------------------------
        # These are only for software testing before the real sensors work.
        #
        # Meaning:
        #   For each stack name ("od", "ou", "pu", "pd"),
        #   store which slot indices actually contain reels.
        #
        # Example:
        #   self.test_reel_slots["od"] = {3}
        # means:
        #   when the robot scans orange-down stack, slot 3 should appear occupied.
        #
        # Another example:
        #   self.test_reel_slots["ou"] = {2, 6}
        # means:
        #   stack "ou" contains reels at slots 2 and 6.
        #
        # Valid values:
        #   empty set      -> no reels in that stack
        #   set of ints    -> occupied 1-based slot indices in that stack
        #
        # This is used only by branch_has_reel().
        #
        # Current branch-routing test case:
        #   node 5   -> pd slot 2
        #   node 16  -> pu slot 2
        #   node 24  -> ou slot 2
        #   node 28  -> ou slot 6
        self.test_reel_slots = {
            "od": set(),
            "ou": {2, 6},
            "pu": {2},
            "pd": {2},
        }

        # Meaning:
        #   For each stack, what colour the reel should be classified as.
        #
        # Example:
        #   self.test_reel_colour["od"] = "red"
        #
        # Valid values:
        #   None, "blue", "green", "yellow", "red"
        #
        # This is only used as fallback in identify_picked_reel()
        # if real resistance measurement is not implemented yet.
        #
        # IMPORTANT:
        #   This fallback is per-stack, not per-slot.
        #   That is fine for branch-routing tests where the main goal is to
        #   verify that the robot turns into the correct branches.
        self.test_reel_colour = {
            "od": None,
            "ou": "yellow",
            "pu": "green",
            "pd": "blue",
        }

        # ------------------------------------------------------------------
        # BUTTON INPUT SETUP
        # ------------------------------------------------------------------
        # Start button:
        #   physical pin 21 = GP16
        #
        # Configured as digital input with pull-down resistor.
        # Expected behaviour:
        #   released -> 0
        #   pressed  -> 1
        self.start_button = Pin(16, Pin.IN, Pin.PULL_DOWN)

        # Stop button:
        #   not wired yet
        #
        # When you have a real stop button, create it here in the same way,
        # for example:
        #   self.stop_button = Pin(X, Pin.IN, Pin.PULL_DOWN)
        self.stop_button = None

    # ---------------- start / stop ----------------
    def start_pressed(self):
        """
        PURPOSE:
            Tell the mission runner whether the start button has been pressed.

        WHEN IT IS USED:
            Called once at startup in _maybe_wait_for_start().
            The robot waits until this returns True.

        WHAT YOU NEED TO FILL IN:
            Read the real start button input pin.
            Usually this is a digital input:
                - pressed -> True
                - not pressed -> False

        WHAT IT MUST RETURN:
            True  -> start mission now
            False -> keep waiting

        IMPORTANT:
            Return a real bool.
            Do not return integers, strings, etc.
        """
        return bool(self.start_button.value())

    def stop_pressed(self):
        """
        PURPOSE:
            Emergency/manual stop input.

        WHEN IT IS USED:
            Called every loop in mission_runner_v2.
            If it returns True, the robot goes to STOP.

        WHAT YOU NEED TO FILL IN:
            Read the real stop button input pin.

        WHAT IT MUST RETURN:
            True  -> force robot to stop immediately
            False -> continue normal operation
        """
        if self.stop_button is None:
            return False
        return bool(self.stop_button.value())

    # ---------------- scanning ----------------
    def branch_has_reel(self, stack_name, slot_index, side):
        """
        PURPOSE:
            Detect whether there is a reel present in the currently scanned branch.

        WHEN IT IS USED:
            During perimeter scanning.
            When the robot reaches a scan point, mission_runner calls this.
            If this returns True, the robot leaves the main route and starts GRAB.

        INPUTS:
            stack_name:
                Which stack is being scanned.
                One of: "od", "ou", "pu", "pd"

            slot_index:
                Which branch position within that stack is being scanned.
                1-based integer.

            side:
                Which side sensor should be used to check this branch.
                Usually "left" or "right".
                This comes from the planner metadata.

        WHAT YOU NEED TO FILL IN:
            Use the correct side-facing distance sensor for that branch.
            Example logic:
                - read left or right distance sensor depending on `side`
                - compare measured distance to threshold
                - if object is closer than expected empty-branch distance,
                  return True

            Typical rule:
                branch occupied -> distance small enough -> True
                branch empty    -> distance too large   -> False

        WHAT IT MUST RETURN:
            True  -> a reel is present in that branch
            False -> no reel is present

        IMPORTANT:
            This should be a clean yes/no decision.
            Do not return the measured distance itself.
        """
        occupied_slots = self.test_reel_slots.get(stack_name, set())
        return slot_index in occupied_slots

    # ---------------- pickup ----------------
    def pickup_target_reached(self, stack_name):
        """
        PURPOSE:
            Tell the grab logic when the robot has driven far enough into the reel
            branch and should stop approaching.

        WHEN IT IS USED:
            During GRAB_FORWARD in grab_movement().
            The robot keeps driving forward until either:
                - this returns True
                - or a timeout is reached

        INPUTS:
            stack_name:
                Which stack branch we are currently entering.
                You may or may not need this in the real implementation.

        WHAT YOU NEED TO FILL IN:
            Use the front-facing approach sensor, probably ultrasonic or ToF.

            Example logic:
                - read front distance to reel
                - if distance <= desired grab distance:
                      return True
                  else:
                      return False

            The desired grab distance is the distance where the gripper is aligned
            correctly with the reel for pickup.

        WHAT IT MUST RETURN:
            True  -> stop forward approach now
            False -> keep moving forward

        IMPORTANT:
            This is NOT "did we grab it?"
            This is only "have we reached the correct pickup position?"
        """
        # Branch-routing test mode:
        # immediately report that the target position has been reached,
        # so the robot does not depend on any real pickup sensor.
        return True

    def close_gripper(self):
        """
        PURPOSE:
            Physically close or actuate the grabbing mechanism.

        WHEN IT IS USED:
            Called once in GRAB_WAIT after the robot has stopped at the reel.

        WHAT YOU NEED TO FILL IN:
            Put the real actuator command here.
            Examples:
                - set servo angle to closed position
                - run DC motor for clamp closing
                - energise gripper actuator
                - lower arm and close claw if your mechanism needs that

        WHAT IT MUST RETURN:
            Preferably return nothing (None).

        IMPORTANT:
            This function is an ACTION, not a sensor read.
            It should COMMAND hardware.
            The code does not currently rely on a return value.

            If your grab takes time, you have two options:
                1. issue the command here and rely on GRAB_WAIT_MS
                2. later extend the state machine with more grab substates
        """
        # Branch-routing test mode:
        # ignore real grabber hardware for now.
        return None

    def reel_secured(self):
        """
        PURPOSE:
            Check whether the reel is actually held securely after closing gripper.

        WHEN IT IS USED:
            Currently not used by the provided state machine,
            but it SHOULD be used later for robustness.

        WHAT YOU NEED TO FILL IN:
            Any real confirmation that pickup succeeded.
            Possible sources:
                - microswitch triggered by closed reel
                - motor current spike profile
                - beam break
                - distance sensor showing reel now attached
                - contact sensor on clamp

        WHAT IT MUST RETURN:
            True  -> reel is successfully secured
            False -> pickup failed / reel not secure

        IMPORTANT:
            This is very useful and should probably be integrated into grab logic later.
            Right now it is just a placeholder hook.
        """
        # Branch-routing test mode:
        # always report success so the mission can continue.
        return True

    def measure_reel_resistance(self):
        """
        PURPOSE:
            Measure the resistance of the picked reel.

        WHEN IT IS USED:
            Called inside identify_picked_reel() after grabbing.

        WHAT YOU NEED TO FILL IN:
            Implement the real electrical measurement method.
            That depends on your hardware design, for example:
                - analog voltage divider
                - ADC measurement across known resistor
                - direct ohmmeter-like setup
                - contact pads touching reel terminals

            You should:
                1. take the raw reading
                2. convert it into resistance in ohms
                3. return the numeric resistance value

        WHAT IT MUST RETURN:
            One of:
                - float resistance in ohms
                - int resistance in ohms
                - None if measurement failed / not available

        GOOD EXAMPLE RETURNS:
            47.8
            120.0
            998.2
            None

        IMPORTANT:
            Do NOT return a colour here.
            This function should return the measured electrical value only.
        """
        return None

    def classify_resistance(self, resistance_ohms):
        """
        PURPOSE:
            Convert a measured resistance value into the reel colour class.

        WHEN IT IS USED:
            Called by identify_picked_reel() after measure_reel_resistance().

        INPUTS:
            resistance_ohms:
                Numeric measured resistance, or possibly None.

        WHAT YOU NEED TO FILL IN:
            Use your competition thresholds / calibration bands.

            Example structure:
                if resistance_ohms is None:
                    return None
                elif low_blue <= resistance_ohms < high_blue:
                    return "blue"
                elif ...
                    return "green"
                ...

            You must choose the real threshold ranges from calibration.

        WHAT IT MUST RETURN:
            Exactly one of:
                "blue"
                "green"
                "yellow"
                "red"
                None   (if measurement invalid / ambiguous)

        IMPORTANT:
            Return exactly those lowercase strings, because the planner expects them.
        """
        return None

    def identify_picked_reel(self, stack_name):
        """
        PURPOSE:
            One convenience function that performs reel identification.

        WHEN IT IS USED:
            Called after grabbing, before delivery mission is built.

        WHAT IT DOES:
            1. measure resistance
            2. classify resistance into a colour
            3. if real classification fails, fall back to test_reel_colour

        WHAT YOU NEED TO FILL IN:
            Usually you may not need to change this method much.
            If measure_reel_resistance() and classify_resistance() are implemented,
            this will work automatically.

        WHAT IT MUST RETURN:
            One of:
                "blue", "green", "yellow", "red", or None

        IMPORTANT:
            The mission runner uses this output to decide where to deliver the reel.
            If it returns None, the runner currently stops.
        """
        resistance = self.measure_reel_resistance()
        colour = self.classify_resistance(resistance)
        if colour is not None:
            return colour
        return self.test_reel_colour.get(stack_name)

    # ---------------- placement ----------------
    def drop_target_reached(self, colour_name):
        """
        PURPOSE:
            Tell the place logic when the robot has driven far enough into the
            destination bay and should stop before releasing the reel.

        WHEN IT IS USED:
            During PLACE_FORWARD in place_movement().

        INPUTS:
            colour_name:
                Which colour bay we are entering:
                "blue", "green", "yellow", or "red"

        WHAT YOU NEED TO FILL IN:
            Use whatever sensor tells you the robot is correctly positioned for drop-off.
            This may be:
                - front ultrasonic / ToF
                - physical stop switch
                - known timed distance only (less reliable)
                - marker sensor

            Example logic:
                if front_distance <= desired_drop_distance:
                    return True
                else:
                    return False

        WHAT IT MUST RETURN:
            True  -> stop forward approach and release reel
            False -> keep moving forward

        IMPORTANT:
            This is only about position for drop-off, not whether the reel was released.
        """
        # Branch-routing test mode:
        # immediately report that drop position is reached,
        # so placement does not depend on real sensors.
        return True

    def open_gripper(self):
        """
        PURPOSE:
            Physically release the reel at the destination bay.

        WHEN IT IS USED:
            Called once in PLACE_WAIT.

        WHAT YOU NEED TO FILL IN:
            Put the real actuator command here.
            Examples:
                - open servo claw
                - reverse clamp motor
                - release latch
                - lower and open mechanism, depending on design

        WHAT IT MUST RETURN:
            Preferably None.

        IMPORTANT:
            Like close_gripper(), this is an ACTION function.
            The code does not depend on any return value.
        """
        # Branch-routing test mode:
        # ignore real grabber hardware for now.
        return None

    def reel_released(self):
        """
        PURPOSE:
            Check whether the reel has actually been released.

        WHEN IT IS USED:
            Currently not used by the provided state machine,
            but it should be used later for robustness.

        WHAT YOU NEED TO FILL IN:
            Some real confirmation that the reel is no longer held.
            Examples:
                - gripper switch no longer pressed
                - object sensor says empty
                - clamp current drops
                - distance/contact check

        WHAT IT MUST RETURN:
            True  -> reel successfully released
            False -> reel still stuck / release failed

        IMPORTANT:
            This is another hook that is worth integrating later.
        """
        # Branch-routing test mode:
        # always report success so the mission can continue.
        return True

    # ---------------- indicator LEDs ----------------
    def show_detected_colour(self, colour_name):
        """
        PURPOSE:
            Display the identified reel colour on the LED indicators.

        WHEN IT IS USED:
            Called after identification in the grab sequence.

        INPUTS:
            colour_name:
                One of:
                    "blue", "green", "yellow", "red"
                Possibly None if identification failed.

        WHAT YOU NEED TO FILL IN:
            Drive the real LED outputs accordingly.

            Example:
                - blue LED on for blue
                - green LED on for green
                - red+green for yellow
                - etc., depending on your hardware

        WHAT IT MUST RETURN:
            Preferably None.

        IMPORTANT:
            This is only an output/display function.
            It should not decide anything about navigation.
        """
        return None