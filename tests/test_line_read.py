from machine import Pin
import time

PIN = 2  
s = Pin(PIN, Pin.IN)

while True:
    v = s.value()
    print(v)  # expect: white=1, black=0 (per handout)
    time.sleep(0.05)