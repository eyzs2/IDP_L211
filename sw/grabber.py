from machine import Pin, ADC, PWM
from utime import sleep, ticks_diff, ticks_ms
from pushbutton_logic import stop_function

from line_logic import LEFT, RIGHT, NO_TURN, T, FORWARD, REVERSE

VOLTAGE_THRESHOLD = 30

BLUE = 0
BLUE_THRESHOLD = [3100, 3200] # insert range of voltage values
GREEN = 1
GREEN_THRESHOLD = [2700, 2800]
RED = 2
RED_THRESHOLD = [1800, 1950]
YELLOW = 3
YELLOW_THRESHOLD = [200, 400]


LED = []

BOTTOM_RACK = 0
TOP_RACK = 1
TRANSPORT = 2

bot_angle = 100 # (+35 deg from neutral)
top_angle = 90 # (-35 deg from neutral)
travel_angle = 90


class Servo:
    def __init__(self, pin, min_us=500, max_us=2500, max_angle=270):
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(50)

        self.min_us = min_us
        self.max_us = max_us
        self.max_angle = max_angle

        self.period = 20000  # 20 ms in microseconds

    def angle(self, angle):

        if angle < 0:
            angle = 0
        if angle > self.max_angle:
            angle = self.max_angle

        pulse = self.min_us + (angle / self.max_angle) * (self.max_us - self.min_us)

        self.pwm.duty_ns(int(pulse * 1000))

    def off(self):
        self.pwm.deinit()
            

class Grabber:
    def __init__(self, resistance_pin, grab_pin, tilt_pin):
        self.resistanceSensor = ADC(resistance_pin)
        self.grabServo = Servo(grab_pin)
        self.grabServo.angle(185)
        self.grabTilt = Servo(tilt_pin)
        self.grabTilt.angle(90)

    def reel_identifier(self): # detect reel via pot div, to discuss with electrical 
        # light up respective LED
        mvoltList = []
        for i in range(3):
            mvolts = int(self.resistanceSensor.read_u16() * (1/65535) * 3300)
            mvoltList.append(mvolts)
            print(mvolts)
            sleep(0.5)
        mvolts = sum(mvoltList)/len(mvoltList)
        print(mvolts)
        if BLUE_THRESHOLD[0] <= mvolts <= BLUE_THRESHOLD[1]:
            print('blue')
            return BLUE
        elif GREEN_THRESHOLD[0] <= mvolts <= GREEN_THRESHOLD[1]:
            print('green')
            return GREEN
        elif RED_THRESHOLD[0] <= mvolts <= RED_THRESHOLD[1]:
            print('red')
            return RED
        elif YELLOW_THRESHOLD[0] <= mvolts <= YELLOW_THRESHOLD[1]:
            print('yellow')
            return YELLOW
        print('nothing')
        return None
    
    def grabber_align(self,level=TRANSPORT):
        if level == BOTTOM_RACK:
            self.grabTilt.angle(bot_angle)
        elif level == TOP_RACK:
            self.grabTilt.angle(top_angle)
        else:
            self.grabTilt.angle(travel_angle)

    def pickup(self):
        grab_angle = 185 
        while grab_angle >= 160:
            stop_function()
            self.grabServo.angle(grab_angle)
            grab_angle -= 5
            # if (self.mvolts >= VOLTAGE_THRESHOLD):
            #     break
            sleep(0.5)
        # reel_id = self.reel_identifier()
        # return reel_id
    
    def dropoff(self):
        self.grabServo.angle(185)



def test_grabber():
    grabber = Grabber(28, 15, 13)
    # start = ticks_ms()
    # straight ahead angle for grab servo = 180
    # grabber.grabServo.angle(200)

    grabber.grabTilt.angle(100)

    # test_angles = [0, 45, 90, 135, 180]

    # while ticks_diff(ticks_ms(), start) < 15000:
    #     # PWM the LED
    #     for angle in test_angles:
    #         grabber.grabTilt.angle(angle)
    #         # grabber.grabTilt.duty_u16(u16_level)
    #         sleep(2)
    #         print('tilt angle ', angle)

    # # grabber.grabServo.angle(180)
    # grabber.grabTilt.angle(180)
    sleep(1)
    grabber.grabServo.angle(160)
    sleep(1)

    nice = grabber.reel_identifier()

    # grabber.grabTilt.angle(90)
    # sleep(1)

    # grabber.grabTilt.angle(135)
    # sleep(1)

   


if __name__ == '__main__':
    test_grabber()

        


