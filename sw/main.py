from machine import Pin
from utime import sleep, ticks_ms, ticks_diff
from test_motor import Motor


from line_logic import LineSensor
from start_box import exit_start_box
from test_motor import Motor

LEFT = 0
RIGHT = 1

MODE = "A"  # "A" = left at first T then rights after, "B" = right at first T then lefts after.
loop_mode = 1 if MODE == "A" else 0

# change to actual pins
BUTTON_PIN = 12
LED_PIN = 28

# motor pins and sensor pins (change to actual)
LEFT_ON_PIN = 26
RIGHT_ON_PIN = 27
LEFT_TURN_PIN = 28
RIGHT_TURN_PIN = 29

LEFT_MOTOR_DIR = 4
LEFT_MOTOR_PWM = 5
RIGHT_MOTOR_DIR = 6
RIGHT_MOTOR_PWM = 7


button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_DOWN)
led = Pin(LED_PIN, Pin.OUT)

line = LineSensor(leftOnPin=LEFT_ON_PIN, rightOnPin=RIGHT_ON_PIN,
                  leftTurnPin=LEFT_TURN_PIN, rightTurnPin=RIGHT_TURN_PIN)

motors = [
    Motor(dirPin=LEFT_MOTOR_DIR, PWMPin=LEFT_MOTOR_PWM),
    Motor(dirPin=RIGHT_MOTOR_DIR, PWMPin=RIGHT_MOTOR_PWM),
]

def reset_memory():
    global racks_visited, current_rack
    racks_visited = []
    current_rack = None

def wait_for_button():
    while button.value() == 0:
        sleep(0.01)
    sleep(0.15)     # debounce
    while button.value() == 1:
        sleep(0.01)

while True:
    # 1) wait for press
    wait_for_button()

    # 2) reset
    reset_memory()

    # 3) exit start box
    ok = exit_start_box(line, motors, speed=35)
    if not ok:
        # if could not find the line; stop and wait for another push button press
        motors[LEFT].off()
        motors[RIGHT].off()
        continue

    # 4) now do line logic code for the rest
    while True:
        led.toggle()
        line.lineFollow(motors, loop=loop_mode)

        # allows the button to cause stop and restart mid-run
        if button.value() == 1:
            # break out to outer loop to reset + restart
            sleep(0.15)
            while button.value() == 1:
                sleep(0.01)
            motors[LEFT].off()
            motors[RIGHT].off()
            break
