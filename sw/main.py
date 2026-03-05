# actions:
# 1) wait for push button press.
# 2) reset stored memory.
# 3) drive out of the start box until front sensors detect line.
# 4) begin normal line-follow logic.
# 5) if button pressed again during run, stop motors and restart from step 1.


from machine import Pin
from utime import sleep
from motor import Motor
from line_logic import LineSensor, LEFT, RIGHT, T, NO_TURN
from start_box import exit_start_box
from pushbutton_logic import ButtonEdge, request_stop_irq, StopRequested
from line_following_test import run_line_following_test
from test_linelogic import lineLogicTest


# Mode A = Turn LEFT at first T junction
# Mode B = Turn RIGHT at first T junction
from reelsensor import ReelSensor

MODE = "A"
loop_mode = RIGHT if MODE == "A" else LEFT
LEFT_MOTOR = 0
RIGHT_MOTOR = 1

# pin configs (need editing based on electrical teams decisions):

BUTTON_PIN = 15
led = Pin("LED", Pin.OUT)

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


# memory variables to store state about the mission:

racks_visited = []
current_rack = None

# Flag to signal immediate stop during line follow
STOP_REQUESTED = False

# Push button (active HIGH with PULL_DOWN)
button_pin = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_DOWN)
button = ButtonEdge(button_pin, debounce_ms=150)
# Set up interrupt on button pin for immediate stop during run


# Line sensor object (from line_logic.py)
line = LineSensor(
    leftOnPin=LEFT_ON_PIN,
    rightOnPin=RIGHT_ON_PIN,
    leftTurnPin=LEFT_TURN_PIN,
    rightTurnPin=RIGHT_TURN_PIN
)

# Motor objects
motors = [
    Motor(dirPin=LEFT_MOTOR_DIR, PWMPin=LEFT_MOTOR_PWM),
    Motor(dirPin=RIGHT_MOTOR_DIR, PWMPin=RIGHT_MOTOR_PWM),
]

# Reel sensor object
reel = ReelSensor(
    leftReelSensorPin=LEFT_REEL_SENSOR_PIN,
    rightReelSensorPin=RIGHT_REEL_SENSOR_PIN
)


# helper functions
def reset_memory():
    # clears all stored mission data, called every time the button is pressed.
    global racks_visited, current_rack

    print("Resetting robot memory...")

    racks_visited = []
    current_rack = None

    print("Memory reset complete.")


def stop_motors():
    motors[LEFT_MOTOR].off()
    motors[RIGHT_MOTOR].off()


def wait_for_button_press():
    print("Waiting for button press...")
    while not button.pressed():
        sleep(0.01)
    print("Button pressed.")

# Set up interrupt on button pin for immediate stop during run
button_pin.irq(trigger=Pin.IRQ_RISING, handler=request_stop_irq)


# main program loop

while True:

    # 1) WAIT FOR BUTTON PRESS TO START / RESTART
    wait_for_button_press()
    
    # Reset stop flag at start of each run
    STOP_REQUESTED = False

    # 2) RESET MEMORY AND STOP MOTORS
    reset_memory()
    stop_motors()

    try:
        lineLogicTest()
    except StopRequested:
        print("Stop requested — stopping motors and restarting.")
        stop_motors()
        # small delay to debounce / give scheduled exceptions time to clear
        sleep(0.1)
        continue
    



    # print("Starting robot from black box...")

    # # 3) EXIT BLACK START BOX
    # # drives straight forward until BOTH front sensors detect the line.
    # # exit_start_box() returns:
    # # True for successfully found the line
    # # False for timeout / failure

    # success = exit_start_box(line, motors, motorspeed=35)

    # if not success:
    #     print("Failed to detect line. Waiting for restart.")
    #     stop_motors()
    #     continue   # go back to waiting for button


    # print("Line detected. Beginning line-following mode.")

    # # 4) NORMAL LINE FOLLOW MODE
    # # This continues until the button is pressed again.

    # win = run_line_following_test(motors, line)
        # Check if stop was requested via button interrupt
        # if STOP_REQUESTED:
        #     print("Restart requested.")
        #     stop_motors()
        #     break  # exit inner loop and restart from top

        # # calling line-follow logic
        # line.turnLogic(turnDirection=loop_mode, motors=motors)
        # line.lineFollow(motors)

        # # flash LED to indicate robot is active
        # led.toggle()

        # sleep(0.01)
