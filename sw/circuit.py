from machine import Pin
from utime import sleep, ticks_diff, ticks_ms
from line_logic import LineSensor, LEFT, RIGHT, NO_TURN, T, FORWARD, REVERSE
from motor import Motor
from start_box import exit_start_box
from reelsensor import ReelSensor
from pushbutton_logic import stop_function
from grabber import Grabber

travel_time = 2000

BUTTON_PIN = 15
# led = Pin("LED", Pin.OUT)

# Front line sensors
LEFT_ON_PIN = 13
RIGHT_ON_PIN = 12

# Rear line sensors
LEFT_TURN_PIN = 27
RIGHT_TURN_PIN = 28

# Motor pins
LEFT_MOTOR_DIR = 4
LEFT_MOTOR_PWM = 5
RIGHT_MOTOR_DIR = 7
RIGHT_MOTOR_PWM = 6

# Reel sensor pins (analog inputs for distance sensors)
LEFT_REEL_SENSOR_PIN = 0  # adjust based on electrical team's wiring
RIGHT_REEL_SENSOR_PIN = 1  # adjust based on electrical team's wiring

LEFT_REEL_SDA_PIN = 8
LEFT_REEL_SCL_PIN = 9
RIGHT_REEL_SDA_PIN = 10
RIGHT_REEL_SCL_PIN = 11

GRABBER_RESISTANCE_PIN = 19 # update!!!
GRABBER_GRAB_SERVO_PIN = 15
GRABBER_TILT_SERVO_PIN = 13

motors = [
    Motor(dirPin=LEFT_MOTOR_DIR, PWMPin=LEFT_MOTOR_PWM),
    Motor(dirPin=RIGHT_MOTOR_DIR, PWMPin=RIGHT_MOTOR_PWM),
]

# Line sensor object (from line_logic.py)
line = LineSensor(
    leftOnPin=LEFT_ON_PIN,
    rightOnPin=RIGHT_ON_PIN,
    leftTurnPin=LEFT_TURN_PIN,
    rightTurnPin=RIGHT_TURN_PIN,
    motors=motors
)


# Reel sensor object
reel = ReelSensor(
    leftReelSDA=LEFT_REEL_SDA_PIN,
    leftReelSCL=LEFT_REEL_SCL_PIN,
    rightReelSDA=RIGHT_REEL_SDA_PIN,
    rightReelSCL=RIGHT_REEL_SCL_PIN
)

grabber = Grabber(
    resistance_pin = GRABBER_RESISTANCE_PIN,
    grab_pin=GRABBER_GRAB_SERVO_PIN,
    tilt_pin=GRABBER_TILT_SERVO_PIN
)


def main_loop(side, line: LineSensor, reel: ReelSensor, grabber):
    reelMode = False
    while True:
        stop_function()
        while reelMode:
            stop_function()
            line.lineFollow()
            if line.turnSense[side].value():
                for motor in line.motors:
                    motor.off()
                if not reel.check_reel_detected(side):
                    print('no reel detected')
                    while line.turnSense[side].value():
                        line.lineFollow()
                    continue
                else:
                    # line.turnLogic(side)
                    # travel_start = ticks_ms()
                    # while ticks_diff(ticks_ms(),travel_start) > travel_time: # adjust travel_time based on distance to reel
                    #     line.lineFollow()
                    # for motor in line.motors:
                    #     motor.off()
                    reel.grab(line, grabber, side)
                    reelMode = False

        line.lineFollow()
        if line.leftTurn.value() and line.rightTurn.value():
            reelMode = True
            print("reel mode activcated")
        line.turnLogic(side)

if __name__ == "__main__":
    main_loop(RIGHT, line, reel, grabber)



                
            