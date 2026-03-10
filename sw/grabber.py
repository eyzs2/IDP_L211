from machine import Pin, ADC, PWM
from utime import sleep, ticks_diff, ticks_ms
from pushbutton_logic import stop_function

from line_logic import LEFT, RIGHT, NO_TURN, T, FORWARD, REVERSE

RESISTANCE_THRESHOLD = 10

BLUE = 0
BLUE_THRESHOLD = [] # insert range of voltage values
GREEN = 1
GREEN_THRESHOLD = []
YELLOW = 2
YELLOW_THRESHOLD = []
RED = 3
RED_THRESHOLD = []

BOTTOM_RACK = 0
TOP_RACK = 1

bot_angle = 170 # (+35 deg from neutral)
top_angle = 100 # (-35 deg from neutral)

class Grabber:
    def __init__(self, resistance_pin, grab_pin, tilt_pin):
        self.resistanceSensor = ADC(resistance_pin)
        self.grabServo = PWM(Pin(grab_pin), freq=1000)
        self.grabServo.duty_u16(0)
        self.grabTilt = PWM(Pin(tilt_pin), freq=1000)
        self.grabTilt.duty_u16(int(65535*0.5)) 

    def reel_identifier(self): # detect reel via pot div, to discuss with electrical 
        # light up respective LED
        volts = self.resistanceSensor.read_u16()
        if BLUE_THRESHOLD[0] <= volts <= BLUE_THRESHOLD[1]:
            return BLUE
        return None
    
    def grabber_align(self,level):
        if level == BOTTOM_RACK:
            self.grabTilt.duty_u16(int(65535 * bot_angle/270))
        else:
            self.grabTilt.duty_u16(int(65535 * top_angle/270))

    def pickup(self):
        grab_angle = 0
        while self.resistanceSensor.read_u16() < RESISTANCE_THRESHOLD:
            stop_function()
            self.grabServo.duty_u16(int(65535 * (grab_angle/270)))
            grab_angle += 2
            sleep(0.1)
        reel_id = self.reel_identifier()
        return reel_id



def test_grabber():
    grabber = Grabber(19, 13, 15)
    level = 0  # 0-100
    direction = 1  # 1=up, -1=down
    start = ticks_ms()

    while ticks_diff(ticks_ms(), start) < 15000:
        # PWM the LED
        u16_level = int(65535 * level / 270)
        grabber.grabServo.duty_u16(u16_level)
        # grabber.grabTilt.duty_u16(u16_level)
    
        # update level and sleep
        print(f"Angle={level}, u16_level={u16_level}, direction={direction}")
        level += direction
        if level == 30:
            direction = -1
        elif level == 0:
            direction = 1
        sleep(0.1)

if __name__ == '__main__':
    test_grabber()

        


