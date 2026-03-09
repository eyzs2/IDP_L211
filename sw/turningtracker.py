from utime import sleep, ticks_diff, ticks_ms
from line_logic import LineSensor, LEFT, RIGHT, NO_TURN, T
from start_box import exit_start_box
from reelsensor import ReelSensor

def _stop_motors(motors):
    motors[LEFT].off()
    motors[RIGHT].off()
    
def run_turning_tracker(motors, line: LineSensor):
    rightTurns = {1, 9, 11, 19}
    leftTurns = {2}

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

    DEBOUNCE_S = 0.03
    CLEAR_CONFIRM_MS = 120

    # helpers
    def rear_L():
        return line.turnSense[LEFT].value() == 1

    def rear_R():
        return line.turnSense[RIGHT].value() == 1

    # main loop
    while True:
        line.lineFollow(DIRECTION)

        L = rear_L()
        R = rear_R()

        # -------- Detect & count events (only once per physical junction) --------
        event_fired = False
        event_type = None

        # T event first (priority)
        if (not lockout_T) and L and R:
            sleep(DEBOUNCE_S)
            if rear_L() and rear_R():
                t_count += 1
                right_any_count += 1
                left_any_count += 1
                lockout_T = True
                lockout_R = True
                lockout_L = True
                clear_T_start = clear_R_start = clear_L_start = None

                event_fired = True
                event_type = "T"

        # Right-only event
        elif (not lockout_R) and R:
            sleep(DEBOUNCE_S)
            if rear_R():
                right_any_count += 1
                lockout_R = True
                clear_R_start = None

                event_fired = True
                event_type = "R"

        # Left-only event
        elif (not lockout_L) and L:
            sleep(DEBOUNCE_S)
            if rear_L():
                left_any_count += 1
                lockout_L = True
                clear_L_start = None

                event_fired = True
                event_type = "L"

        # -------- Unlock logic --------
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

        # -------- Print + scheduled turns ONLY when a new event fired --------
        if event_fired:
            print(
                "EVENT", event_type,
                "| right_any =", right_any_count,
                "| left_any =", left_any_count,
                "| T =", t_count
            )

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