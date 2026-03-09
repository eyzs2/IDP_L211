from utime import sleep
from line_logic import LineSensor, LEFT, RIGHT, NO_TURN, T
from start_box import exit_start_box
from reelsensor import ReelSensor


class TurnScheduler:
    """ 
    rules for the LINE FOLLOWING TEST:

    1) Turn LEFT at first T
    2) Turn RIGHT at second T
    2) Turn RIGHT at the next 2 right corner junctions defined by:
         - both front sensors black
         - rear-right sensor white
    3) Then turn RIGHT at the 8th right-corner opportunity after that.
    4) Then turn LEFT at the 2nd left-corner opportunity after that.
    5) After that final left completes, drive forward for 0.5s ignoring sensors, then stop.
    """

    def __init__(self, line: LineSensor):
        self.line = line
        self.stage = 0
        self.corner_taken = 0 
        self.lockout = False
        self.final_drive_pending = False

    def _front_both_black(self) -> bool:
        return (self.line.leftOn.value() == 0) and (self.line.rightOn.value() == 0)

    def _rear_right_white(self) -> bool:
        return self.line.rightTurn.value() == 1

    def _rear_left_white(self) -> bool:
        return self.line.leftTurn.value() == 1

    def _is_right_corner(self, detection) -> bool:
        return (detection == RIGHT) and self._front_both_black() and self._rear_right_white()

    def _is_left_corner(self, detection) -> bool:
        return (detection == LEFT) and self._rear_left_white()

def _stop_motors(motors):
    motors[LEFT].off()
    motors[RIGHT].off()


def run_line_following_test(motors, line: LineSensor, reel_sensor: ReelSensor = None):
    """
    Main line following loop with reel detection at right junctions.
    Reel mode only activates after the first right turn is completed.
    
    Args:
        motors: Motor objects [LEFT, RIGHT]
        line: LineSensor object
        reel_sensor: ReelSensor object (optional)
    
    Returns:
        True if mission completed successfully
    """
    
    loop_mode = RIGHT  # RIGHT for loop A

    line.turnLogic(turnDirection = LEFT, motors=motors)
    print("first left complee")

    rightCounter = 0
    leftCounter = 0

    rightTurns = {1, 9, 11, 19}
    leftTurns = {2}
    
    # Flag to track if we've completed the first right turn
    first_right_turn_complete = False

    while True:
        line.lineFollow(motors, 0)
        
        # Check for right junctions/turns
        if line.turnSense[RIGHT].value():
            sleep(0.15)
            if line.turnSense[RIGHT].value():
                print("turn ", RIGHT, " detected")
                rightCounter += 1
                print("turn number ", rightCounter)
                
                # Check for reel ONLY if first right turn is complete
                # In Mode A, only check right side reel sensor
                if reel_sensor and first_right_turn_complete:
                    if reel_sensor.check_reel_detected(RIGHT):
                        print("Reel found!")
                        _stop_motors(motors)
                        sleep(0.5)
                        
                        # Turn toward the reel (right side in Mode A)
                        print(f"Turning toward reel on side {RIGHT}")
                        line.turnLogic(turnDirection=RIGHT, motors=motors)
                        
                        # Resume line following
                        print("Reel grabbed, resuming line follow")
                        sleep(1.0)
                
                # Execute turn if it's in our turn schedule
                if rightCounter in rightTurns:
                    _stop_motors(motors)
                    print("turning")
                    line.turnLogic(turnDirection=RIGHT, motors=motors)
                    
                    # Mark first right turn as complete after executing turn #1
                    if rightCounter == 1:
                        first_right_turn_complete = True
                        print("First right turn complete - enabling reel detection")

        # Check for left junctions/turns (only after we've completed enough right turns)
        if rightCounter == max(rightTurns):
            if line.turnSense[LEFT].value():
                sleep(0.15)
                if line.turnSense[LEFT].value():
                    leftCounter += 1
                    print(f"left detection {leftCounter}")
                    if leftCounter in leftTurns:
                        _stop_motors(motors)
                        print("turning left")
                        line.turnLogic(turnDirection=LEFT, motors=motors)
                        break

    # Final drive sequence
    while not (line.turnSense[LEFT].value() and line.turnSense[RIGHT].value()):
        line.lineFollow(motors)
    
    _stop_motors(motors)

    motors[LEFT].Forward(side=LEFT, speed=60)
    motors[RIGHT].Forward(side=RIGHT, speed=60)

    sleep(1)  # ignore all sensors
    _stop_motors(motors)
    return True
