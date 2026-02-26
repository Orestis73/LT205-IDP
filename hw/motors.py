from machine import Pin, PWM

#function to keep value of speed between -1 and 1
#if user gives value outside this range, it will be set to the closest limit (-1 or 1)
def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

#motor class to control a single DC motor with direction and PWM speed control
class DCMotor:

    def __init__(self, dir_pin: int, pwm_pin: int, pwm_freq_hz: int = 1000, invert: bool = False):
        self._dir = Pin(dir_pin, Pin.OUT) #set motor direction pin
        self._pwm = PWM(Pin(pwm_pin)) #set pwm pin, scale between 0 (off) and 65535 (full on)
        self._pwm.freq(pwm_freq_hz)
        self._invert = invert #if true, then the motor direction is inverted
        self.stop()

    def stop(self):
        self._pwm.duty_u16(0) #set duty cycle to 0 (off), meaning turn off the motor

    def set(self, speed: float):
 
        speed = _clamp(speed, -1.0, 1.0) #ensure speed is between -1 and 1
        if self._invert:
            speed = -speed

        if speed >= 0:
            self._dir.value(0)  # forward
            duty = int(65535 * speed)
        else:
            self._dir.value(1)  # reverse
            duty = int(65535 * (-speed))

        self._pwm.duty_u16(duty)

#convenience class to control both motors together, e.g. for arcade drive
class MotorPair: 
    
    #left and right are DCMotor instances
    def __init__(self, left: DCMotor, right: DCMotor):
        self.left = left
        self.right = right

    def stop(self):
        self.left.stop()
        self.right.stop()

    #set left and right motor speeds together, with values between -1 and 1
    def set_left_right(self, left_speed: float, right_speed: float):
        self.left.set(left_speed)
        self.right.set(right_speed)

    #arcade drive: throttle controls forward/backward, steer controls left/right turning
    def arcade(self, throttle: float, steer: float):

        throttle = _clamp(throttle, -1.0, 1.0)
        steer = _clamp(steer, -1.0, 1.0)

        left = throttle - steer
        right = throttle + steer

        # normalise if needed to keep within [-1,1]
        m = max(abs(left), abs(right), 1.0) 
        self.set_left_right(left / m, right / m)
