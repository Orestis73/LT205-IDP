from machine import Pin, PWM
from utime import sleep
import config

def test_pwm():
    pwm_pin_no = 15  # Pin 28 = GP28 (labelled 34 on the jumper)
    pwm_pin = PWM(Pin(pwm_pin_no), 100)

    level = 0  # 0-100
    degree = 0
    direction = 1  # 1=up, -1=down



class grabber:
    def _init_ (self,pin1,pin2):
        self.servo_pin1=pin1
        self.servo_pin2=pin2
        self.pwm_pin1 = PWM(Pin(self.servo_pin1),100)
        self.pwm_pin2 = PWM(Pin(self.servo_pin2),100)
        self.servo1_deg=0
        self.servo2_deg=0
        
    def write_servo1(self, deg):
        u16_level1 = int (9500+deg*48.5)
        self.pwm_pin1.duty_u16(u16_level1)
        self.servo1_deg=deg
        
    def write_servo2(self, deg):
        u16_level2 = int (9800+deg*48.5)
        self.pwm_pin2.duty_u16(u16_level2)
        self.servo2_deg=deg
    
    def reset(self):
        self.write_servo1(0)
        self.write_servo2(0)
    
    def grab(self, lift=True):
        self.write_servo1(12)
        if lift:
            self.write_servo2(self.servo2_deg-5)
    
    def opn(self, lift=True):
        if lift:
            self.write_servo2(-5)
        self.write_servo1(-10)
    
    def lift(self,deg):
        self.write_servo2(-deg)
            
        
        
grabber = grabber(15,13)
grabber.reset()
sleep(0.2)
grabber.opn(False)
sleep(1)
grabber.grab(True)
sleep(0.5)
grabber.lift(17)
sleep(2)
grabber.reset()
grabber.opn(False)
#sleep(1)
#grabber.opn(False)
#sleep(1)
#grabber.reset()