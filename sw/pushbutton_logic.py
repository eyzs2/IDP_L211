from machine import Pin
from utime import sleep, ticks_ms, ticks_diff

# change based on pin config
LED_PIN = 28
BUTTON_PIN = 12

led = Pin(LED_PIN, Pin.OUT)
button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_DOWN)

running = False

# memory variables (replace with diff names once written)
racks_visited = []
current_rack = None
loop_mode = 0


# state reset function 
def reset_robot_state():
    global racks_visited, current_rack, loop_mode, running

    print("Resetting robot...")

    # Clear stored mission data
    racks_visited = []
    current_rack = None
    loop_mode = 0

    led.value(0)

    running = False

    print("Reset complete")


# start function
def start_robot():
    global running

    print("Starting robot from black box...")
    running = True

    # Call your separate black start box logic here
    # Example:
    # black_box_start_sequence()

# main loop
last_button_state = 0
debounce_time = 200  # ms
last_press_time = 0

while True:

    current_button_state = button.value()
    now = ticks_ms()

    # Rising edge detection (button press)
    if current_button_state == 1 and last_button_state == 0:
        if ticks_diff(now, last_press_time) > debounce_time:

            last_press_time = now

            # always treat button press as full reset + restart
            reset_robot_state()
            start_robot()

    last_button_state = current_button_state

    # running state
    if running:
        led.toggle()

        # main robot logic here
        # e.g.:
        # line.lineFollow(motors, loop_mode)

        sleep(0.3)

    else:
        sleep(0.05)