from machine import Pin
from utime import sleep, ticks_diff, ticks_ms
from pushbutton_logic import stop_function

# Line functionality --> going on line from start box (initial state), staying on line (steady state), junction logic (interrupt)

LEFT = 0
RIGHT = 1
NO_TURN = 2
T = 3

FORWARD = 0
REVERSE = 1


class LineSensor:
    def __init__(self, leftOnPin, rightOnPin, leftTurnPin, rightTurnPin, motors):
        self.leftOn = Pin(leftOnPin, Pin.IN, Pin.PULL_DOWN)
        self.rightOn = Pin(rightOnPin, Pin.IN, Pin.PULL_DOWN)
        self.leftTurn = Pin(leftTurnPin, Pin.IN, Pin.PULL_DOWN)
        self.rightTurn = Pin(rightTurnPin, Pin.IN, Pin.PULL_DOWN)
        self.motors = motors
        self.lineSense = [self.leftOn, self.rightOn]
        self.turnSense = [self.leftTurn, self.rightTurn]
        self.loopCompletion = False
   

    # function to ensure robot stays on course following the line
    def lineFollow(self, direction = FORWARD):
        stop_function()
        motors = self.motors
        lineSense = self.lineSense

        left_on = lineSense[LEFT].value()
        right_on = lineSense[RIGHT].value()

        # sets base speeds for motors 
        fast_speed = 80 
        slow_speed = 35

        # chooses movement functions based on direction
        if direction == REVERSE:
            def fast(side):
                motors[side].Reverse(side=side, speed=slow_speed)

            def slow(side):
                motors[side].Reverse(side=side, speed=15)

            def opposite(side):
                motors[side].Forward(side=side, speed=slow_speed)
        else:
            def fast(side):
                motors[side].Forward(side=side, speed=fast_speed)

            def slow(side):
                motors[side].Forward(side=side, speed=slow_speed)

            def opposite(side):
                motors[side].Reverse(side=side, speed=slow_speed)

        # both sensors on line, travels straight at fast speed
        if left_on and right_on:
            fast(LEFT)
            fast(RIGHT)

        # left off, correct by slowing right
        elif not left_on and right_on:
            fast(LEFT)
            slow(RIGHT)

        # left on, correct by slowing left
        elif left_on and not right_on:
            slow(LEFT)
            fast(RIGHT)

      #   both off, travel in opp direction for 0.5 seconds
        else:
            opposite(LEFT)
            opposite(RIGHT)
            sleep(0.1)
            stop_function()
            return

        sleep(0.01)


    # function to execute turns at junctions based on turn tracker
    def turnLogic(self, turnDirection):
        stop_function()
        motors = self.motors
        turnSense = self.turnSense

        turnDetection = NO_TURN

        # detecting T-junctions 
        if turnSense[LEFT].value() and turnSense[RIGHT].value():
            sleep(0.05)
            if turnSense[LEFT].value() and turnSense[RIGHT].value(): #verification
                turnDetection = T

        # detection of left or right only turns 
        for i in range(len(turnSense)):
            if turnSense[i].value() and not turnSense[(i+1)%2].value(): # If one side turn is detected, verify after T
                sleep(0.05)
                if turnSense[i].value():
                    turnDetection = i # else treat as corner
                    break 

        if turnDetection != NO_TURN:
            print("turn detected", "turn type: ", turnDetection)
            stop_function()

            if turnDetection == T or turnDetection == turnDirection:
                motors[LEFT].off()
                motors[RIGHT].off()
                sleep(0.5)  # brief pause to let robot stop before turning

                # execute turn based on predetermined outcome (loop)
                print("turning, ", "type: ", turnDirection)

                motors[turnDirection].Reverse(side=turnDirection, speed=30)  # change turn speed here as needed
                motors[(turnDirection+1) % 2].Forward(side=(turnDirection+1) % 2, speed=70)

                # settle time: don't check sensors yet
                while (self.lineSense[LEFT].value() or self.lineSense[RIGHT].value()):
                    stop_function()
                    sleep(0.1) 
                # wait until BOTH front sensors are back on the line
                print('cleared lines')
                while not (self.lineSense[LEFT].value() and self.lineSense[RIGHT].value()):
                    stop_function()
                    sleep(0.1)
                

                motors[LEFT].off()
                motors[RIGHT].off()
                sleep(0.5)

                print("turn complete")

            else:
                print("non-loop turn detected!")


# def test_line_sensors():
#     # Front line sensors
#     left_on_pin = 26
#     right_on_pin = 21

#     # Rear line sensors
#     left_turn_pin = 27
#     right_turn_pin = 20
#     motors = []
    
#     line = LineSensor(leftOnPin=left_on_pin,
#     rightOnPin=right_on_pin,
#     leftTurnPin=left_turn_pin,
#     rightTurnPin=right_turn_pin,
#     motors=motors)

#     start_time = ticks_ms()

#     while ticks_diff(ticks_ms(), start_time) < 10000:
#         if line.leftOn.value():
#             print('left front on!')

#         if line.rightOn.value():
#             print('right front on!')

#         if line.leftTurn.value():
#             print('left turn')
        
#         if line.rightTurn.value():
#             print('right turn')
            


# if __name__ == '__main__':
#     test_line_sensors()
