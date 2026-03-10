from machine import Pin, PWM
from utime import sleep

class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)  # set motor direction pin
        self.pwm = PWM(Pin(PWMPin))  # set motor pwm pin
        self.pwm.freq(1000)  # set PWM frequency
        self.pwm.duty_u16(0)  # set duty cycle - 0=off
        
    def off(self):
        self.pwm.duty_u16(0)
        
    def Forward(self, side, speed=100):
        self.mDir.value((side+1)%2)                     # forward = 0 reverse = 1 motor
        self.pwm.duty_u16(int(65535 * speed / 100))  # speed range 0-100 motor

    def Reverse(self, side, speed=30):
        self.mDir.value(side)
        self.pwm.duty_u16(int(65535 * speed / 100))


# def test_motor3():
#     motor3 = Motor(dirPin=7, PWMPin=6)  # Motor 3 is controlled from Motor Driv2 #1, which is on GP4/5

#     print("Forward")
#     motor3.Forward(1)
#     sleep(1)
#     print("Reverse")
#     motor3.Reverse(1)
#     sleep(1)
#     motor3.off()



# if __name__ == "__main__":
#     test_motor3()
