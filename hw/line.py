from machine import Pin

class LineSensors:
    # Pins: list of 4 GPIO pin numbers for the line sensors, ordered LEFT->RIGHT
    # invert: True if your sensor polarity is opposite (i.e. black=HIGH, white=LOW)
    def __init__(self, pins, invert=False):
        if len(pins) != 4:
            raise ValueError("Expected 4 line sensor pins (LEFT->RIGHT).")
        self._pins = [Pin(p, Pin.IN) for p in pins]
        self._invert = invert

    # Read raw GPIO values
    def read_raw(self):
        return [p.value() for p in self._pins]

    # Convert raw readings to blackness: 1 = black, 0 = white
    def read_black(self):
        raw = self.read_raw()
        black = [1 - v for v in raw]   # assumes black=LOW, white=HIGH
        if self._invert:
            black = [1 - b for b in black]
        return black

    # Convert blackness to whiteness: 1 = white, 0 = black
    def read_white(self):
        black = self.read_black()
        return [1 - b for b in black]

    # Pack blackness into 4-bit pattern [L, ML, MR, R]
    def read_bits(self):
        b = self.read_black()
        return (b[0] << 3) | (b[1] << 2) | (b[2] << 1) | (b[3] << 0)

    # Count how many sensors see black
    def sum_black(self):
        b = self.read_black()
        return b[0] + b[1] + b[2] + b[3]

    # Count how many sensors see white
    def sum_white(self):
        w = self.read_white()
        return w[0] + w[1] + w[2] + w[3]

    # Centered clean line pattern in blackness terms
    def good_line(self):
        b = self.read_black()
        return (b[0] == 0 and b[1] == 1 and b[2] == 1 and b[3] == 0)

    def is_intersection(self, white=None):
        w = white if white is not None else self.read_white()
        return bool(w[0] and w[3])

    def is_corner_raw(self, white=None, sumw=None):
        w = white if white is not None else self.read_white()
        sw = sumw if sumw is not None else (w[0] + w[1] + w[2] + w[3])

        # Matches 0111 or 1110 in white, but not 1111 and not 0110
        return (sw == 3) and ((w[0] ^ w[3]) == 1)

    def sense(self):
        black = self.read_black()
        white = [1 - b for b in black]
        sumw = white[0] + white[1] + white[2] + white[3]
        good_line = (black[0] == 0 and black[1] == 1 and black[2] == 1 and black[3] == 0)
        return black, white, sumw, good_line