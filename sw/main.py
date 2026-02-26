from utime import sleep_ms

from line_logic import LineSensor
from startfinish_box import BlackBoxController
from test_motor import Motor 

# change to actual pins once determined 
LEFT_MOTOR_DIR = 4
LEFT_MOTOR_PWM = 5
RIGHT_MOTOR_DIR = 6
RIGHT_MOTOR_PWM = 7

LEFT_ON_PIN = 26
RIGHT_ON_PIN = 27
LEFT_TURN_PIN = 28
RIGHT_TURN_PIN = 29

def main():
    # Motors as we've defined: motor[0] = left, motor[1] = right
    motor = [
        Motor(dirPin=LEFT_MOTOR_DIR, PWMPin=LEFT_MOTOR_PWM),
        Motor(dirPin=RIGHT_MOTOR_DIR, PWMPin=RIGHT_MOTOR_PWM),
    ]

    # Line sensor object (from line_logic)
    line = LineSensor(
        leftOnPin=LEFT_ON_PIN,
        rightOnPin=RIGHT_ON_PIN,
        leftTurnPin=LEFT_TURN_PIN,
        rightTurnPin=RIGHT_TURN_PIN
    )

    # Start-box controller (from startfinish_box logic)
    bb = BlackBoxController(
        line_sensor=line,
        motor=motor,
        exit_speed=35,
        confirm_ms_start=100
    )

    # step 1 - EXIT BLACK BOX
    while True:
        if bb.step_start_exit():   # returns True when both front sensors are white (debounced)
            break

    # small pause to prevent overshoot
    sleep_ms(50)

    # step 2 - LINE FOLLOW LOOP
    loop_id = "A"  # change name to whatever we decide for reel detection mode
    while True:
        line.lineFollow(motor=motor, loop=loop_id)
        sleep_ms(10)  # optional, prevents maxing the CPU

if __name__ == "__main__":
    main()


