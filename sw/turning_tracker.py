from utime import sleep, ticks_diff, ticks_ms
from line_logic import LineSensor, LEFT, RIGHT, NO_TURN, T, FORWARD, REVERSE
# from motor import Motor
# from start_box import exit_start_box
# from reelsensor import ReelSensor
from pushbutton_logic import stop_function
from grabber import Grabber

DEBOUNCE_S = 0.05
CLEAR_CONFIRM_MS = 150

def _stop_motors(motors):
    motors[LEFT].off()
    motors[RIGHT].off()
    
def run_turning_tracker(
        motors, 
        line: LineSensor, 
        reel, 
        grabber: Grabber, 
        inputRightTurns = {}, inputLeftTurns = {}, inputReelCheckRights = {}, inputReelCheckLefts = {}):
    rightTurns = inputRightTurns # {2}
    leftTurns = inputLeftTurns # {1}
    reelCheckLefts = inputReelCheckLefts # {}
    reelCheckRights = inputReelCheckRights # {3, 4, 5, 6, 7, 8}
    

    # counts
    right_any_count = 0   # RIGHT sensor high (includes T)
    left_any_count = 0    # LEFT sensor high (includes T)
    t_count = 0           # both high

    # lockouts so we don't double-count
    lockout_R = False
    lockout_L = False
    lockout_T = False

    clear_R_start = None
    clear_L_start = None
    clear_T_start = None


    # helpers
    def rear_L():
        return line.turnSense[LEFT].value() == 1

    def rear_R():
        return line.turnSense[RIGHT].value() == 1

    # main loop
    while True:
        stop_function()
        line.lineFollow(FORWARD)

        L = rear_L()
        R = rear_R()

        # Detect & count events only once per physical junction
        event_fired = False
        event_type = None

        # T event first (priority)
        if (not lockout_T) and L and R:
            # sleep(DEBOUNCE_S)
            if rear_L() and rear_R():
                t_count += 1
                right_any_count += 1
                left_any_count += 1
                lockout_T = lockout_R = lockout_L = True
                clear_T_start = clear_R_start = clear_L_start = None

                event_fired = True
                event_type = "T"

        # Right-only event
        elif (not lockout_R) and R:
            # sleep(DEBOUNCE_S)
            if rear_R():
                right_any_count += 1
                lockout_R = True
                clear_R_start = None

                event_fired = True
                event_type = "R"

        # Left-only event
        elif (not lockout_L) and L:
            # sleep(DEBOUNCE_S)
            if rear_L():
                left_any_count += 1
                lockout_L = True
                clear_L_start = None

                event_fired = True
                event_type = "L"

        # Unlock T when not (L and R) for long enough
        if lockout_T:
            if not (rear_L() and rear_R()):
                if clear_T_start is None:
                    clear_T_start = ticks_ms()
                elif ticks_diff(ticks_ms(), clear_T_start) > CLEAR_CONFIRM_MS:
                    lockout_T = False
                    clear_T_start = None
            else:
                clear_T_start = None

        # Unlock R when R low for long enough
        if lockout_R:
            if not rear_R():
                if clear_R_start is None:
                    clear_R_start = ticks_ms()
                elif ticks_diff(ticks_ms(), clear_R_start) > CLEAR_CONFIRM_MS:
                    lockout_R = False
                    clear_R_start = None
            else:
                clear_R_start = None

        # Unlock L when L low for long enough
        if lockout_L:
            if not rear_L():
                if clear_L_start is None:
                    clear_L_start = ticks_ms()
                elif ticks_diff(ticks_ms(), clear_L_start) > CLEAR_CONFIRM_MS:
                    lockout_L = False
                    clear_L_start = None
            else:
                clear_L_start = None


        # Print + scheduled turns ONLY when a new event fired 
        if event_fired:
            if left_any_count in reelCheckLefts:
                print("REEL CHECK at left count: ", left_any_count)
                _stop_motors(motors)
                sleep(0.2) # might need to adjust

                if reel.check_reel_detected(LEFT):
                    print("REEL DETECTED - starting grab")
                    # reelCheckRights.remove(right_any_count)
                    reel.grab(line, grabber, LEFT)
                    
                    sleep(0.1) # might need to adjust

                    run_dropoff_tracker(motors, line, grabber, LEFT)
                    run_return_to_start(motors, line, LEFT)
                    break
                else:
                    print("No reel found")
                    # reelCheckRights.remove(right_any_count)
                    sleep(0.01)
            if right_any_count in reelCheckRights:
                print("REEL CHECK at right count: ", right_any_count)
                _stop_motors(motors)
                sleep(0.2) # might need to adjust

                if reel.check_reel_detected(RIGHT):
                    print("REEL DETECTED - starting grab")
                    # reelCheckRights.remove(right_any_count)
                    reel.grab(line, grabber, RIGHT)
                    
                    sleep(0.1) # might need to adjust
                    run_dropoff_tracker(motors, line, grabber, RIGHT)
                    run_return_to_start(motors, line, RIGHT)
                    break
                else:
                    print("No reel found")
                    # reelCheckRights.remove(right_any_count)
                    sleep(0.01) # might need to adjust

            # only turn on scheduled numbers
            if right_any_count in rightTurns:
                print("SCHEDULE: RIGHT turn at", right_any_count)
                _stop_motors(motors)
                line.turnLogic(turnDirection=RIGHT)
                sleep(0.1) # give time to clear turn before next event

            if left_any_count in leftTurns:
                print("SCHEDULE: LEFT turn at", left_any_count)
                _stop_motors(motors)
                line.turnLogic(turnDirection=LEFT)
                sleep(0.1) # give time to clear turn before next event


def run_dropoff_tracker(motors, line: LineSensor, grabber: Grabber, side):
    t_count = 0
    lockout_T = False
    clear_T_start = None

    def rear_L():
        return line.turnSense[LEFT].value() == 1

    def rear_R():
        return line.turnSense[RIGHT].value() == 1
    while True:
        stop_function() 
        line.lineFollow(FORWARD)

        L = rear_L()
        R = rear_R()

        event_fired = False

        if (not lockout_T) and L and R:
            if rear_L() and rear_R():
                t_count += 1
                lockout_T = True
                clear_T_start = None
                event_fired = True
                print("T detected:", t_count)

        if lockout_T:
            if not (rear_L() and rear_R()):
                if clear_T_start is None:
                    clear_T_start = ticks_ms()
                elif ticks_diff(ticks_ms(), clear_T_start) > CLEAR_CONFIRM_MS:
                    lockout_T = False
                    clear_T_start = None
            else:
                clear_T_start = None

        if event_fired:
            if t_count == 1:
                print("first T reached - turning left")
                _stop_motors(motors)
                line.turnLogic(turnDirection=((side+1)%2))
                sleep(0.1)

            elif t_count == 2:
                _stop_motors(motors)
                print("drop off occurring")
                grabber.dropoff()
                break



def run_return_to_start(motors, line: LineSensor, side):
    left_any_count = 0
    lockout_L = False
    clear_L_start = None

    def rear_L():
        return line.turnSense[LEFT].value() == 1

    while True:
        stop_function()
        line.lineFollow(FORWARD)

        event_fired = False

        # detect left event once
        if (not lockout_L) and rear_L():
            if rear_L():
                left_any_count += 1
                lockout_L = True
                clear_L_start = None
                event_fired = True
                print("Return left detected:", left_any_count)

        # unlock left after it has been clear for long enough
        if lockout_L:
            if not rear_L():
                if clear_L_start is None:
                    clear_L_start = ticks_ms()
                elif ticks_diff(ticks_ms(), clear_L_start) > CLEAR_CONFIRM_MS:
                    lockout_L = False
                    clear_L_start = None
            else:
                clear_L_start = None

        if event_fired:
            # first left: take it
            if left_any_count == 1:
                print("Return route: taking first left")
                _stop_motors(motors)
                line.turnLogic(turnDirection=LEFT)
                sleep(0.1)

            # second left: ignore it
            elif left_any_count == 2:
                print("Return route: ignoring second left")

            # third left: take it, then forward 1 second, then dance
            elif left_any_count == 3:
                print("Return route: taking third left")
                _stop_motors(motors)
                line.turnLogic(turnDirection=LEFT)
                sleep(0.1)

                print("driving forward into finish area")
                start_time = ticks_ms()
                while ticks_diff(ticks_ms(), start_time) < 1000:
                    stop_function()
                    line.lineFollow(FORWARD)

                _stop_motors(motors)
                sleep(0.2)

                do_dance(motors)
                break


def do_dance(motors):
    print("starting dance")

    start_time = ticks_ms()
    while ticks_diff(ticks_ms(), start_time) < 5000:
        stop_function()
        motors[LEFT].Reverse(side=LEFT, speed=90)
        motors[RIGHT].Forward(side=RIGHT, speed=90)

    _stop_motors(motors)
    print("dance complete")