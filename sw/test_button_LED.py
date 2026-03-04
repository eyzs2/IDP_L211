from machine import Pin, reset
from utime import sleep

#Set the LED pin and configuration
led_pin = 28
led = Pin(led_pin, Pin.OUT)

#Set the button pin
button_pin = 15
button = Pin(button_pin, Pin.IN, Pin.PULL_DOWN)

#Continiously update the LED value and print said value

if __name__ == '__main__':
    while True:
        if button.value():
            reset()
            break
