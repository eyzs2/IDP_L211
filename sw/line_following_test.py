# line_follow_test_main.py
from machine import Pin
from utime import sleep, ticks_ms, ticks_diff

from line_logic import LineSensor, LEFT, RIGHT, NO_TURN, T  


BUTTON_PIN = 15  # <-- set to our push button GPIO

# Sensor pins (choose from our wiring)
LEFT_ON_PIN = 26
RIGHT_ON_PIN = 27
LEFT_TURN_PIN = 28
RIGHT_TURN_PIN = 29

# Motor pins (choose from our wiring)
LEFT_MOTOR_DIR = 4
LEFT_MOTOR_PWM = 5
RIGHT_MOTOR_DIR = 6
RIGHT_MOTOR_PWM = 7

# Perimeter rule:
# allow ONLY the 4 perimeter turns (rectangle corners), ignore all other turn detections.
# Simplest: "take the first 4 *corner* turns we see, ignore the rest"
# (If your track has more turn markers on the inside, this works well.)
MAX_PERIMETER_TURNS = 4


def wait_for_button_press(btn: Pin):
    # assumes active-low with pull-up; if yours is active-high, flip condition.
    while btn.value() == 1:
        sleep(0.01)
    # debounce
    sleep(0.15)
    while btn.value() == 0:
        sleep(0.01)


def drive_out_of_black_box_until_on_line(line: LineSensor, motors, timeout_s=5):
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < int(timeout_s * 1000):
        # drive forward slowly
        motors[LEFT].Forward(30)
        motors[RIGHT].Forward(30)

        if line.lineSense[LEFT].value() and line.lineSense[RIGHT].value():
            motors[LEFT].off()
            motors[RIGHT].off()
            sleep(0.05)
            return True

        sleep(0.01)

    # timeout fail-safe
    motors[LEFT].off()
    motors[RIGHT].off()
    return False


def make_turn_filter(line: LineSensor, max_turns: int):
    """
    Returns a function that replaces line.turnLogic.
    It will:
      - allow a turn only for the first `max_turns` *corner* detections (LEFT or RIGHT)
      - ignore T-junctions (returns NO_TURN for T)
      - after turns are used up, suppress all turning
    """
    original_turnLogic = line.turnLogic
    turns_taken = {"n": 0}

    def filtered_turnLogic(motor, loop):
        # Ask the original code what it thinks is happening
        detection = original_turnLogic(motor, loop)

        # Ignore T-junction logic entirely for this perimeter test
        if detection == T:
            return NO_TURN

        # Corner detections are LEFT (0) or RIGHT (1)
        if detection in (LEFT, RIGHT):
            if turns_taken["n"] < max_turns:
                turns_taken["n"] += 1
                return detection
            else:
                return NO_TURN

        return NO_TURN

    return filtered_turnLogic


def main():
    button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)
    line = LineSensor(LEFT_ON_PIN, RIGHT_ON_PIN, LEFT_TURN_PIN, RIGHT_TURN_PIN)
    motors = [
        Motor(dirPin=LEFT_MOTOR_DIR, PWMPin=LEFT_MOTOR_PWM),
        Motor(dirPin=RIGHT_MOTOR_DIR, PWMPin=RIGHT_MOTOR_PWM),
    ]

    # 1) Wait for button
    wait_for_button_press(button)

    # 2) Exit black box to reach the line
    ok = drive_out_of_black_box_until_on_line(line, motors)
    if not ok:
        # couldn't find line: stop
        motors[LEFT].off()
        motors[RIGHT].off()
        while True:
            sleep(1)

    # 3) Install the "ignore internal junctions" filter
    line.turnLogic = make_turn_filter(line, MAX_PERIMETER_TURNS)

    # 4) Follow line forever (or until you decide to stop)
    loop = 0 
    while True:
        line.lineFollow(motors, loop)
        sleep(0.01)


if __name__ == "__main__":
    main()