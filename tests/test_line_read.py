import utime
import config
from hw.line import LineSensors


def main():
    #Test reading line sensors and printing blackness values
    line = LineSensors(config.LINE_PINS, invert=config.LINE_INVERT)

    #Continuously read and print blackness values every 100ms
    while True:
        b = line.read_black()
        print(f"Blackness: {b}")
        utime.sleep_ms(100)

if __name__ == "__main__":
    main()
