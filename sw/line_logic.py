from machine import Pin
from utime import sleep

# Line functionality --> going on line from start box (initial state), staying on line (steady state), junction logic (interrupt)


def offLine(p):
    value = p.value()
    print(f"Input changed, value={value}")

class LineSensor:
    def __init__(self, leftOnPin, rightOnPin, leftTurnPin, rightTurnPin):
        self.leftOn = Pin(leftOnPin, Pin.IN, Pin.PULL_DOWN)
        self.rightOn = Pin(rightOnPin, Pin.IN, Pin.PULL_DOWN)
        self.leftTurn = Pin(leftTurnPin, Pin.IN, Pin.PULL_DOWN)
        self.rightTurn = Pin(rightTurnPin, Pin.IN, Pin.PULL_DOWN)
        self.lineSense = [self.leftOn, self.rightOn]
        self.turnSense = [self.leftTurn, self.rightTurn]
        # self.leftOn.irq(handler=offLine, trigger=Pin.IRQ_FALLING)
        # self.rightOn.irq(handler=offLine, trigger=Pin.IRQ_FALLING)
        

    def lineFollow(self, motor, loop): # define 0 left, 1 right
        # Takes in Motor list argument for correction

        lineSense = self.lineSense

        if not lineSense[0].value() and not lineSense[1].value():
            motor[0].Reverse()
            motor[1].Reverse()
            sleep(0.5)
        else: 
            for i in range(len(lineSense)):
                if not lineSense[i].value():
                    motor[(i+1)%2].off()
                    sleep(0.1)
        
        self.turnLogic(motor=motor, loop=loop)

        motor[0].Forward()
        motor[1].Forward()
            
        

    def turnLogic(self, motor, loop):

        turnSense = self.turnSense

        if turnSense[0] and turnSense[1]:
            # T junction, hardcode based on loop A/B
            return
                
        # TODO
        # if either turn sensor is off , sleep 0.2s, test again
        # if one still off, execute turn logic based on sensor
        # if both off, execute hardcoded t-junction logic


# 



# def Line():
    # leftonLinePin = 26
    # rightonLinePin = 27
#     leftonLine = Pin(leftonLinePin, Pin.IN, Pin.PULL_DOWN)
#     leftonLine.irq(handler=offLine)
#     rightonLine = Pin(rightonLinePin, Pin.IN, Pin.PULL_DOWN)
#     rightonLine.irq(handler=offLine)




    