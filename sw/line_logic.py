from machine import Pin
from utime import sleep

# Line functionality --> going on line from start box (initial state), staying on line (steady state), junction logic (interrupt)

LEFT = 0
RIGHT = 1
NO_TURN = 2
T = 3



class LineSensor:
    def __init__(self, leftOnPin, rightOnPin, leftTurnPin, rightTurnPin):
        self.leftOn = Pin(leftOnPin, Pin.IN, Pin.PULL_DOWN)
        self.rightOn = Pin(rightOnPin, Pin.IN, Pin.PULL_DOWN)
        self.leftTurn = Pin(leftTurnPin, Pin.IN, Pin.PULL_DOWN)
        self.rightTurn = Pin(rightTurnPin, Pin.IN, Pin.PULL_DOWN)
        self.lineSense = [self.leftOn, self.rightOn]
        self.turnSense = [self.leftTurn, self.rightTurn]
        self.loopCompletion = False
        # self.leftOn.irq(handler=offLine, trigger=Pin.IRQ_FALLING)
        # self.rightOn.irq(handler=offLine, trigger=Pin.IRQ_FALLING)
        

    def lineFollow(self, motors): # define 0 left, 1 right
        # Takes in Motor list argument for correction
        # stop_check: optional callable that returns True if should stop immediately

        # once loop done, self.loopCompletion = True
        # Decided by if all racks are checked
        print("Starting line follow")

        lineSense = self.lineSense
        
            # Check if either sensor is off the line
        if not lineSense[LEFT].value() or not lineSense[RIGHT].value():
            print("correcting")
            # Find which sensor is off
            for i in range(len(lineSense)):
                if not lineSense[i].value():
                    # Turn down opposite side speed only until BOTH sensors back on
                    opposite = (i+1) % 2
                    while not (lineSense[LEFT].value() and lineSense[RIGHT].value()):
                        # if stop_check and stop_check():
                        #     motors[LEFT].off()
                        #     motors[RIGHT].off()
                        #     return
                        motors[i].Forward(side=i, speed=60)  # retain other side speed
                        motors[opposite].Forward(side=opposite, speed=40)  # Reduced speed on opposite
                        sleep(0.01)
                    break

        motors[LEFT].Forward(side=LEFT, speed=60)
        motors[RIGHT].Forward(side=RIGHT, speed=60)
        sleep(0.01)

    def turnLogic(self, turnDirection, motors):

        turnSense = self.turnSense

        turnDetection = NO_TURN

        if turnSense[LEFT].value() and turnSense[RIGHT].value():
            sleep(0.1)
            if turnSense[LEFT].value() and turnSense[RIGHT].value(): #verification
                turnDetection = T

        for i in range(len(turnSense)):
            if turnSense[i].value() and not turnSense[(i+1)%2].value(): # If one side turn is detected, verify after T
                sleep(0.1)
                if turnSense[i].value():
                    turnDetection = i # else treat as corner
            
        if turnDetection != NO_TURN:
            # if override != None: # if override present, turn direction set as override
            #     turnDirection = override
            
            print("turn detected", "turn type: ", turnDetection)
            if turnDetection == T or turnDetection == turnDirection:
                motors[LEFT].off()
                motors[RIGHT].off()
                sleep(2)  # brief pause to let robot stop before turning
                # execute turn based on predetermined outcome (loop)
                print("turning ", "type: ", turnDirection)
                motors[turnDirection].Forward(side=turnDirection, speed=5)  # change turn speed here as needed
                motors[(turnDirection+1) % 2].Forward(side=(turnDirection+1) % 2, speed=80)

                # settle time: don't check sensors yet
                while (self.lineSense[LEFT].value() or self.lineSense[RIGHT].value()):
                    sleep(0.1) 
                # wait until BOTH front sensors are back on the line
                while not (self.lineSense[LEFT].value() and self.lineSense[RIGHT].value()):
                    # if stop_check and stop_check():
                    #     motors[LEFT].off()
                    #     motors[RIGHT].off()
                    #     return
                    sleep(0.01)

                motors[LEFT].off()
                motors[RIGHT].off()
                sleep(1.0)

                print("turn complete")
            else:
                print("non-loop turn detected!!")




        # lineSense is a list with left and right front sensors; lineSense[n].value() gives value of corresponding sensor
        # if not lineSense[LEFT].value() and not lineSense[RIGHT].value(): # End of junction/completely off line
        #     if turnDetection == NO_TURN:
        #         motors[LEFT].Reverse(side=LEFT,speed=10)
        #         motors[RIGHT].Reverse(side=RIGHT,speed=10)
        #         sleep(0.5)


