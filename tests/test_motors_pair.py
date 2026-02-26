from utime import sleep
import config
from hw.motors import DCMotor, MotorPair

def main():
    #Initialize motors
    left = DCMotor(config.MOTOR_LEFT_DIR_PIN, config.MOTOR_LEFT_PWM_PIN, config.MOTOR_PWM_FREQ_HZ, config.MOTOR_LEFT_INVERT)
    right = DCMotor(config.MOTOR_RIGHT_DIR_PIN, config.MOTOR_RIGHT_PWM_PIN, config.MOTOR_PWM_FREQ_HZ, config.MOTOR_RIGHT_INVERT)
    drive = MotorPair(left, right)

    #Test motor control by running forward, reverse, and turns
    print("Forward")
    drive.set_left_right(0.3, 0.3); sleep(1)
    print("Stop")
    drive.stop(); sleep(0.5)
    print("Reverse")
    drive.set_left_right(-0.3, -0.3); sleep(1)
    print("Turn left")
    drive.set_left_right(-0.25, 0.25); sleep(1)
    print("Turn right")
    drive.set_left_right(0.25, -0.25); sleep(1)

    drive.stop()
    print("Done")

if __name__ == "__main__":
    main()
