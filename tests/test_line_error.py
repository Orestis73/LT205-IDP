# tests/test_line_error.py
from utime import sleep_ms
import config
from hw.line import LineSensors
from control.line_error import error_from_black

def bits4(x):
    x &= 0xF
    s = ""
    for k in (3, 2, 1, 0):
        s += "1" if (x & (1 << k)) else "0"
    return s

def main():
    sensors = LineSensors(config.LINE_PINS, invert=getattr(config, "LINE_INVERT", False))
    while True:
        black = sensors.read_black()
        bits = sensors.read_bits()
        err = error_from_black(black)
        print("bits=", bits4(bits), " black=", black, " err=", err)
        sleep_ms(50)

main()