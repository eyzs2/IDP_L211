from line_logic import LineSensor
from test_motor import Motor
from utime import ticks_ms

from main import LEFT_MOTOR_DIR, LEFT_MOTOR_PWM, RIGHT_MOTOR_DIR, RIGHT_MOTOR_PWM, LEFT_ON_PIN, RIGHT_ON_PIN, LEFT_TURN_PIN, RIGHT_TURN_PIN 


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
    

    while now - start_time < 10000:
        line.lineFollow(motors, loop=1)

if __name__ == "__main__":
    lineLogicTest()