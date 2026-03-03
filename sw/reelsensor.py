from machine import Pin
from utime import sleep

from line_logic import T, LEFT, RIGHT, NO_TURN

THRESHOLD_DIST = 300 #to adjust based on testing

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
        self.reelMode = False

    def reelDetect(self, line, loop, motors):
        while line.turnLogic() != T:
            for i in range(len(line.lineSense)): # If one sensor is off
                if not line.lineSense[i].value() and line.turnDetection == NO_TURN:
                    motors[(i+1)%2].off() # turn off opposite side motor to correct
                    sleep(0.3) #TEST adjust based on tests

            if line.turnLogic() == loop:
                motors[LEFT].off()
                motors(RIGHT).off()
                if self.reelSensors[loop] > THRESHOLD_DIST:
                    # grabber logic
                    print("reel detected")

            motors[LEFT].Forward()
            motors[RIGHT].Forward()


        self.reelMode = False
            
