from machine import Pin
# Line sensor module class to read 4x digital line sensors and convert to blackness values
class LineSensors:
    # Pins: list of 4 GPIO pin numbers for the line sensors, ordered LEFT->RIGHT
    # invert: True if your sensor polarity is opposite (i.e. black=HIGH, white=LOW)
    def __init__(self, pins, invert=False):
        if len(pins) != 4:
            raise ValueError("Expected 4 line sensor pins (LEFT->RIGHT).")
        self._pins = [Pin(p, Pin.IN) for p in pins]
        self._invert = invert

    # Read the raw digital values from the 4 sensors
    # Returns list of 4 ints [0/1] as read from GPIO
    def read_raw(self):
        return [p.value() for p in self._pins]

    # Convert raw readings to blackness:
    #   blackness = 1 if black line detected else 0
    # Default assumes: black=LOW (0), white=HIGH (1), so blackness = 1 - raw
    def read_black(self):
        raw = self.read_raw()
        black = [1 - v for v in raw]  # black=LOW -> 1, white=HIGH -> 0

        # If your module behaves opposite, flip again
        if self._invert:
            black = [1 - b for b in black]

        return black

    # Pack blackness into a 4-bit pattern b3..b0 = [L, ML, MR, R]
    # Example: black=[0,1,1,0] -> bits=0b0110 (=6)
    def read_bits(self):
        b = self.read_black()
        return (b[0] << 3) | (b[1] << 2) | (b[2] << 1) | (b[3] << 0)

    # Count how many sensors see black
    def sum_black(self):
        b = self.read_black()
        return b[0] + b[1] + b[2] + b[3]