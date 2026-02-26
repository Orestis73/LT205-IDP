from machine import Pin, PWM
from utime import sleep

class Motor:
    def _init_(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)  # set motor direction pin
        self.pwm = PWM(Pin(PWMPin))       # set motor pwm pin
        self.pwm.freq(1000)               # set PWM frequency
        self.pwm.duty_u16(0)              # set duty cycle - 0=off
        
    def off(self):
        self.pwm.duty_u16(0)
        
    def Forward(self, speed=100):
        self.mDir.value(0)                # forward = 0 reverse = 1
        self.pwm.duty_u16(int(65535 * speed / 100))  

    def Reverse(self, speed=30):
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))

def test_both_motors():
    # Initialize both 12V motors
    motor3 = Motor(dirPin=4, PWMPin=5)  # Connected to J39
    motor4 = Motor(dirPin=7, PWMPin=6)  # Connected to J40
    
    print("Starting dual motor test...")

    while True:
        print("Both Forward")
        motor3.Forward()
        motor4.Forward()
        sleep(1)
        
        print("Both Reverse")
        motor3.Reverse()
        motor4.Reverse()
        sleep(1)

if _name_ == "_main_":
    test_both_motors()