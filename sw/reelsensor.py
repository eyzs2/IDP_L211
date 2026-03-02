from machine import Pin
from utime import sleep

# class Robot:
#     def __init__(self, lineSensor, reelSensor, motors, grabbers):
#         self.lineSensor = lineSensor
#         self.reelSensor = reelSensor
#         self.motors = motors
#         self.grabbers = grabbers

class ReelSensor:
    def __init__(self, leftReelSensorPin, rightReelSensorPin): # takes in i/o pins for reel sensors
        self.leftReelSensor = Pin(leftReelSensorPin, Pin.IN, Pin.PULL_DOWN)
        self.rightReelSensor = Pin(rightReelSensorPin, Pin.IN, Pin.PULL_DOWN)
        self.reelSensors = [self.leftReelSensor, self.rightReelSensor]