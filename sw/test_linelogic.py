from line_logic import LineSensor
from test_motor import Motor
from utime import ticks_ms, sleep

# Front line sensors
LEFT_ON_PIN = 13
RIGHT_ON_PIN = 12

# Rear line sensors
LEFT_TURN_PIN = 28
RIGHT_TURN_PIN = 27

# Motor pins
LEFT_MOTOR_DIR = 4
LEFT_MOTOR_PWM = 5
RIGHT_MOTOR_DIR = 7
RIGHT_MOTOR_PWM = 6

LEFT_REEL_SENSOR = 0
RIGHT_REEL_SENSOR = 0

LEFT = 0
RIGHT = 1

def lineLogicTest():
    line = LineSensor(leftOnPin=LEFT_ON_PIN,
        rightOnPin=RIGHT_ON_PIN,
        leftTurnPin=LEFT_TURN_PIN,
        rightTurnPin=RIGHT_TURN_PIN
    )

    motors = [
        Motor(dirPin=LEFT_MOTOR_DIR, PWMPin=LEFT_MOTOR_PWM),
        Motor(dirPin=RIGHT_MOTOR_DIR, PWMPin=RIGHT_MOTOR_PWM),
    ]

    start_time = ticks_ms()
    now = ticks_ms()
    motors[LEFT].Forward(side=LEFT, speed=80)
    motors[RIGHT].Forward(side=RIGHT, speed=80)

    while now - start_time < 10000:
        now = ticks_ms()
        print(line.leftOn.value(), line.rightOn.value())
        line.lineFollow(motors, loop=1)

    motors[LEFT].off()
    motors[RIGHT].off()

    print("time up")


if __name__ == "__main__":
    lineLogicTest()