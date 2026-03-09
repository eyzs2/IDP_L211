from utime import sleep, ticks_ms, ticks_diff
from line_logic import LineSensor, FORWARD, REVERSE

LEFT = 0
RIGHT = 1

def exit_start_box(line:LineSensor, motors, motorspeed=35, confirm_ms=120, timeout_ms=4000):

    start_time = ticks_ms()
    confirm_start = None

    while ticks_diff(ticks_ms(), start_time) < timeout_ms:

        # drive straight forward 
        motors[LEFT].Forward(LEFT, speed=motorspeed)
        motors[RIGHT].Forward(RIGHT, speed=motorspeed)

        # read front sensors 
        left_turn_on_line = line.leftTurn.value()
        right_turn_on_line = line.rightTurn.value()

         # both must be true (white)
        if left_turn_on_line and right_turn_on_line:

            if confirm_start is None:
                print("Line detected, confirming...")
                confirm_start = ticks_ms()
                motors[LEFT].off()
                motors[RIGHT].off()

            elif ticks_diff(ticks_ms(), confirm_start) > confirm_ms:
                print("Line confirmed. Exiting start box...")
                while (line.turnSense[LEFT].value() or line.turnSense[RIGHT].value()):
                    line.lineFollow()
                while not (line.turnSense[LEFT].value() and line.turnSense[RIGHT].value()):
                    line.lineFollow()

                motors[LEFT].off()
                motors[RIGHT].off()
                print("T reached, start line follow logic...")
                return True
        else:
            confirm_start = None

        sleep(0.01)

    # Timeout safety
    motors[LEFT].off()
    motors[RIGHT].off()
    print("Timeout: Failed to detect line within time limit.")
    return False


