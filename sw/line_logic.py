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
        

    def lineFollow(self, motors, loop, stop_check=None): # define 0 left, 1 right
        # Takes in Motor list argument for correction
        # stop_check: optional callable that returns True if should stop immediately

        # once loop done, self.loopCompletion = True
        # Decided by if all racks are checked
        print("Starting line follow")

        lineSense = self.lineSense
        turnDetection = self.turnLogic()

        # lineSense is a list with left and right front sensors; lineSense[n].value() gives value of corresponding sensor
        # if not lineSense[LEFT].value() and not lineSense[RIGHT].value(): # End of junction/completely off line
        #     if turnDetection == NO_TURN:
        #         motors[LEFT].Reverse(side=LEFT,speed=10)
        #         motors[RIGHT].Reverse(side=RIGHT,speed=10)
        #         sleep(0.5)
            
        if turnDetection != NO_TURN:
            print("turn detected")
            sleep(0.5)
            if turnDetection == T or turnDetection == loop:
                # execute turn based on predetermined outcome (loop)
                print("turning ", "type: ", loop)
                # Stop motors briefly before pivoting
                motors[LEFT].off()
                motors[RIGHT].off()
                sleep(0.1)  # brief pause to let robot stop before turning
                motors[loop].Reverse(side=loop, speed=35)
                motors[(loop+1)%2].Forward(side=(loop+1)%2, speed=35)
                while not (self.turnSense[loop].value() and lineSense[LEFT].value() and lineSense[RIGHT].value()): 
                    # i.e. wait until turn sensor back ON, both line sensors back on line
                    # do testing to determine reverse/forward values to complete turn and fulfil sensor criteria
                    if stop_check and stop_check():
                        motors[LEFT].off()
                        motors[RIGHT].off()
                        return
                    sleep(0.01)            
                    continue
                print("turn complete")
         
            motors[LEFT].off()
            motors[RIGHT].off()
            sleep(1.0)

            while self.turnSense[LEFT].value() or self.turnSense[RIGHT].value(): #keep moving forward until straight line sense is restored ie no turn detected
                if stop_check and stop_check():
                    motors[LEFT].off()
                    motors[RIGHT].off()
                    return
                motors[LEFT].Forward(LEFT, speed=60)
                motors[RIGHT].Forward(RIGHT, speed=60)
            
            motors[LEFT].off()
            motors[RIGHT].off()
            sleep(1.0)


        else: 
            # Check if either sensor is off the line
            if not lineSense[LEFT].value() or not lineSense[RIGHT].value():
                print("correcting")
                # Find which sensor is off
                for i in range(len(lineSense)):
                    if not lineSense[i].value():
                        # Turn down opposite side speed only until BOTH sensors back on
                        opposite = (i+1) % 2
                        while not (lineSense[LEFT].value() and lineSense[RIGHT].value()):
                            if stop_check and stop_check():
                                motors[LEFT].off()
                                motors[RIGHT].off()
                                return
                            motors[opposite].Forward(side=opposite, speed=40)  # Reduced speed on opposite
                            sleep(0.01)
                        break

        motors[LEFT].Forward(side=LEFT, speed=60)
        motors[RIGHT].Forward(side=RIGHT, speed=60)
        sleep(0.1)

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





