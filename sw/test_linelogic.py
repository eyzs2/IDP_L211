from line_logic import LineSensor
from motor import Motor
from utime import ticks_ms, sleep

BUTTON_PIN = 22
# led = Pin("LED", Pin.OUT)

# Front line sensors
LEFT_ON_PIN = 26
RIGHT_ON_PIN = 21

# Rear line sensors
LEFT_TURN_PIN = 27
RIGHT_TURN_PIN = 20

# Motor pins
LEFT_MOTOR_DIR = 4
LEFT_MOTOR_PWM = 5
RIGHT_MOTOR_DIR = 7
RIGHT_MOTOR_PWM = 6

# Reel sensor pins (analog inputs for distance sensors)
# adjust based on electrical's decisions and testing of reel sensor functionality
LEFT_REEL_SDA_PIN = 8
LEFT_REEL_SCL_PIN = 9
RIGHT_REEL_SDA_PIN = 10
RIGHT_REEL_SCL_PIN = 11

LEFT = 0
RIGHT = 1
FORWARD = 0
REVERSE = 1

def lineLogicTest():
    motor = [
        Motor(dirPin=LEFT_MOTOR_DIR, PWMPin=LEFT_MOTOR_PWM),
        Motor(dirPin=RIGHT_MOTOR_DIR, PWMPin=RIGHT_MOTOR_PWM),
    ]
    
    line = LineSensor(leftOnPin=LEFT_ON_PIN,
        rightOnPin=RIGHT_ON_PIN,
        leftTurnPin=LEFT_TURN_PIN,
        rightTurnPin=RIGHT_TURN_PIN,
        motors = motor
    )

    start_time = ticks_ms()
    now = ticks_ms()

    while now - start_time < 15000:
        now = ticks_ms()
        if line.leftTurn.value() or line.rightTurn.value():
            line.turnLogic(turnDirection=4)
        line.lineFollow()

    line.motors[LEFT].off()
    line.motors[RIGHT].off()

    print("time up")


if __name__ == "__main__":
    lineLogicTest()