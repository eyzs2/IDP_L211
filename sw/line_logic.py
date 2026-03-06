from machine import Pin
from utime import sleep
from pushbutton_logic import stop_function

# Line functionality --> going on line from start box (initial state), staying on line (steady state), junction logic (interrupt)

LEFT = 0
RIGHT = 1
NO_TURN = 2
T = 3

FORWARD = 0
REVERSE = 1


class LineSensor:
    def __init__(self, leftOnPin, rightOnPin, leftTurnPin, rightTurnPin):
        self.leftOn = Pin(leftOnPin, Pin.IN, Pin.PULL_DOWN)
        self.rightOn = Pin(rightOnPin, Pin.IN, Pin.PULL_DOWN)
        self.leftTurn = Pin(leftTurnPin, Pin.IN, Pin.PULL_DOWN)
        self.rightTurn = Pin(rightTurnPin, Pin.IN, Pin.PULL_DOWN)
        self.lineSense = [self.leftOn, self.rightOn]
        self.turnSense = [self.leftTurn, self.rightTurn]
        self.loopCompletion = False
   

    def lineFollow(self, motors, direction): # define 0 left, 1 right
        
        stop_function()
        lineSense = self.lineSense
        
        # Check if either sensor is off the line
        if not lineSense[LEFT].value() or not lineSense[RIGHT].value():
            # Find which sensor is off
            for i in range(len(lineSense)):
                stop_function()
                if not lineSense[i].value():
                    # Turn down opposite side speed only until BOTH sensors back on
                    opposite = (i+1) % 2
                    motors[i].Forward(side=i, speed=60)  # retain other side speed
                    motors[opposite].Forward(side=opposite, speed=35)  # Reduced speed on opposite
                    sleep(0.01)

        else:
            if direction == REVERSE:
                motors[LEFT].Reverse(side=LEFT, speed=60)
                motors[RIGHT].Reverse(side=RIGHT, speed=60)
            else:
                motors[LEFT].Forward(side=LEFT, speed=60)
                motors[RIGHT].Forward(side=RIGHT, speed=60)
        sleep(0.01)

    def turnLogic(self, turnDirection, motors):
        stop_function()
        turnSense = self.turnSense

        turnDetection = NO_TURN

        if turnSense[LEFT].value() and turnSense[RIGHT].value():
            sleep(0.05)
            if turnSense[LEFT].value() and turnSense[RIGHT].value(): #verification
                turnDetection = T

        for i in range(len(turnSense)):
            if turnSense[i].value() and not turnSense[(i+1)%2].value(): # If one side turn is detected, verify after T
                sleep(0.05)
                if turnSense[i].value():
                    turnDetection = i # else treat as corner
            
        if turnDetection != NO_TURN:
            # if override != None: # if override present, turn direction set as override
            #     turnDirection = override
            
            print("turn detected", "turn type: ", turnDetection)
            stop_function()
            if turnDetection == T or turnDetection == turnDirection:
                motors[LEFT].off()
                motors[RIGHT].off()
                sleep(1)  # brief pause to let robot stop before turning
                # execute turn based on predetermined outcome (loop)
                print("turning ", "type: ", turnDirection)
                motors[turnDirection].Forward(side=turnDirection, speed=2)  # change turn speed here as needed
                motors[(turnDirection+1) % 2].Forward(side=(turnDirection+1) % 2, speed=80)

                # settle time: don't check sensors yet
                while (self.lineSense[LEFT].value() or self.lineSense[RIGHT].value()):
                    stop_function()
                    sleep(0.1) 
                # wait until BOTH front sensors are back on the line
                while not (self.lineSense[LEFT].value() and self.lineSense[RIGHT].value()):
                    stop_function()
                    sleep(0.01)

                motors[LEFT].off()
                motors[RIGHT].off()
                sleep(1.0)

                print("turn complete")
            else:
                print("non-loop turn detected!")

