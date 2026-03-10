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
from pushbutton_logic import ButtonEdge, request_stop_irq, StopRequested, clear_stop
from test_linelogic import lineLogicTest
from turningtracker import run_turning_tracker

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

LEFT_REEL_SDA = 0 #update UPDATE
LEFT_REEL_SCL = 0 #UPDATE
RIGHT_REEL_SDA = 8
RIGHT_REEL_SCL = 9


# memory variables to store state about the mission:

racks_visited = []
current_rack = None

# Flag to signal immediate stop during line follow
STOP_REQUESTED = False

# Push button (active HIGH with PULL_DOWN)
button_pin = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_DOWN)
button = ButtonEdge(button_pin, debounce_ms=150)
# Set up interrupt on button pin for immediate stop during run

# Motor objects
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
    leftReelSDA=LEFT_REEL_SDA
    leftReelSCL=LEFT_REEL_SCL
    rightReelSDA=RIGHT_REEL_SDA
    rightReelSCL=RIGHT_REEL_SCL
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
    clear_stop()
    sleep(0.5)

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
        # 3) EXIT START BOX
        exit_start_box(line, motors)
        run_turning_tracker(motors, line, reel)
    except StopRequested:
        print("Stop requested — stopping motors and restarting.")
        stop_motors()
        clear_stop()
        # small delay to debounce / give scheduled exceptions time to clear
        sleep(0.3)
        continue
    




