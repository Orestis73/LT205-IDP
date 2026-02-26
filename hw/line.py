from machine import Pin
from typing import List

#Line sensor module class to read 4x digital line sensors and convert to blackness values
class LineSensors:

    #Pins: list of 4 GPIO pin numbers for the line sensors, ordered LEFT->RIGHT
    def __init__(self, pins: List[int], invert: bool = False):
        if len(pins) != 4:
            raise ValueError("Expected 4 line sensor pins (LEFT->RIGHT).")
        self._pins = [Pin(p, Pin.IN) for p in pins]
        self._invert = invert

    #Read the raw digital values from the 4 sensors
    def read_raw(self) -> List[int]:
        return [p.value() for p in self._pins]

    def read_black(self) -> List[int]:
        
        #Convert raw readings to blackness: 1=black, 0=white. Invert if needed.
        raw = self.read_raw()
        black = [1 - v for v in raw]  # black=LOW -> 1
        if self._invert:
            black = [1 - b for b in black]
        return black

    
    