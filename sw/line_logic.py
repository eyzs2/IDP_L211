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
        

    def lineFollow(self, motors, loop): # define 0 left, 1 right
        # Takes in Motor list argument for correction

        # once loop done, self.loopCompletion = True
        # Decided by if all racks are checked

        lineSense = self.lineSense
        turnDetection = self.turnLogic()

        # lineSense is a list with left and right front sensors; lineSense[n].value() gives value of corresponding sensor
        if not lineSense[LEFT].value() and not lineSense[RIGHT].value(): # End of junction/completely off line
            if turnDetection == NO_TURN:
                motors[LEFT].Reverse()
                motors[RIGHT].Reverse()
                sleep(0.5)
            
        if turnDetection != NO_TURN:
            if turnDetection == T or turnDetection == loop:
                # execute turn based on predetermined outcome (loop)
                motors[loop].Reverse()
                motors[(loop+1)%2].Forward()
                while not (self.turnSense[loop].value() and lineSense[LEFT].value() and lineSense[RIGHT].value()): 
                    # i.e. wait until turn sensor back ON, both line sensors back on line
                    # do testing to determine reverse/forward values to complete turn and fulfil sensor criteria
                    sleep(0.01)
                    continue


            # elif turnDetection == loop:
            #     # pivot about centre, inner turn wheel reverse, outer turn forward
            #     motors[turnDetection].Reverse()
            #     motors[(currentTurn+1)%2].Forward()
            #     while self.turnSense[currentTurn].value():
            #         continue
            #     while not (self.turnSense[currentTurn].value() and lineSense[LEFT].value() and lineSense[RIGHT].value()): 
            #         #i.e. wait until turn sensor back ON, both line sensors back on line
            #         #TEST do testing to determine reverse/forward values to complete turn and fulfil sensor criteria
            #         sleep(0.01)
                    continue
            # stop after turn completion

            motors[LEFT].off()
            motors[RIGHT].off()
            sleep(1.0)


           
        else: 
            for i in range(len(lineSense)): # If one sensor is off
                if not lineSense[i].value() and turnDetection == NO_TURN:
                    motors[(i+1)%2].off() # turn off opposite side motor to correct
                    sleep(0.3) #TEST adjust based on tests

        motors[LEFT].Forward()
        motors[RIGHT].Forward()           

    def turnLogic(self):

        turnSense = self.turnSense

        if turnSense[LEFT].value() and turnSense[RIGHT].value():
            return T

        for i in range(len(turnSense)):
            if turnSense[i].value() and not turnSense[(i+1)%2].value(): # If one side turn is detected, check again for T
                sleep(0.2)
                if turnSense[LEFT].value() and turnSense[RIGHT].value(): # Check for T
                    return T
                if turnSense[i].value():
                    return i # else treat as corner
            
        return NO_TURN




    
                
        # TODO
        # if either turn sensor is off , sleep 0.2s, test again
        # if one still off, execute turn logic based on sensor
        # if both off, execute hardcoded t-junction logic



