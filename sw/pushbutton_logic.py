from machine import Pin
from utime import sleep

# Set the LED pin (using LED as a visual indicator to show if robot is on)
led_pin = 28
led = Pin(28, Pin.OUT)

# Set the button pin
button_pin = 12
button = Pin(button_pin, Pin.IN, Pin.PULL_DOWN)

running = False          # Tracks whether program is running
last_button_state = 0    # Start condition

def reset():
    led.value(0) # Pressing button again resets robot and turns off LED

    # ADD CODE HERE to determine what we want resetting (memory of racks etc.)

    print("Reset complete")

while True:
    
    current_button_state = button.value()

    # Detect button press (rising edge)
    if current_button_state == 1 and last_button_state == 0:
        running = not running   # Toggle state
        sleep(0.2)              # Debounce delay
        
        if not running:
            reset()
        else:
            print("Started")

    last_button_state = current_button_state

    # Main program runs only if running is True (i.e. button has been pushed)
    if running:

        # MAIN
        # CODE
        # HERE

        led.toggle() # flash the LED while robot is running
        sleep(0.5)
    else:
        sleep(0.05)