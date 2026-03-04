from utime import sleep
from line_logic import LineSensor, LEFT, RIGHT, NO_TURN, T
from start_box import exit_start_box


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

    # def attach(self):
    #     self._original_turnLogic = self.line.turnLogic
    #     self.line.turnLogic = self.filtered_turnLogic

    # def filtered_turnLogic(self):
    #     detection = self._original_turnLogic()

    #     # clear lockout once no turn is detected anymore
    #     if detection == NO_TURN:
    #         self.lockout = False
    #         return NO_TURN

    #     # prevent double counting at the same physical junction
    #     if self.lockout:
    #         return NO_TURN

    #     # Stage 0: first T junction -> allow T (direction determined by loop=RIGHT)
    #     if self.stage == 0:
    #         if detection == T:
    #             self.lockout = True
    #             self.stage = 1
    #             return LEFT
    #         return NO_TURN

    #     if self.stage == 1:
    #         if detection == T:
    #             self.lockout = True
    #             self.stage = 2
    #             return RIGHT
    #         return NO_TURN

    #     if self.stage == 2:
    #         if self._is_right_corner(detection):
    #             self.lockout = True
    #             self.right_corners_taken += 1
    #             if self.right_corners_taken >= 2:
    #                 self.final_drive_pending = True
    #                 self.stage = 3
    #             return RIGHT
    #         return NO_TURN

    #     # Stage 3: take LEFT on the 2nd left-corner after the 8th right
    #     if self.stage == 3:
    #         if self._is_left_corner(detection):
    #             self.left_corner_count_after_8 += 1
    #             if self.left_corner_count_after_8 == 2:
    #                 self.lockout = True
    #                 self.stage = 4
    #                 self.final_drive_pending = True
    #                 return LEFT
    #         return NO_TURN

    #     # Stage 4: finished, suppress all turns
    #     return NO_TURN


def _stop_motors(motors):
    motors[LEFT].off()
    motors[RIGHT].off()


def run_line_following_test(motors, line: LineSensor):

    # 1) Exit start box
    ok = exit_start_box(line, motors, motorspeed=35, confirm_ms=120, timeout_ms=6000)
    if not ok:
        _stop_motors(motors)
        return False

    # 2) Install filtered turning schedule
    # scheduler = TurnScheduler(line)
    # scheduler.attach()

    loop_mode = RIGHT # RIGHT for loop A

    line.turnLogic(turnDirection = LEFT, motors=motors)

    rightCounter = 0
    leftCounter = 0

    rightTurns = {}

    while True:
        line.lineFollow(motors)
        turnStatus = line.turnLogic(turnDirection = loop_mode, motors=motors)

        # After final left completes, lineFollow returns and we do the final drive
        # if scheduler.final_drive_pending:
        #     scheduler.final_drive_pending = False
        sleep(0.2)  # ignore all sensors
        _stop_motors(motors)
        return True

        sleep(0.01)