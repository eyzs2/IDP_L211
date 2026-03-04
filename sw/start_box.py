from utime import sleep, ticks_ms, ticks_diff

LEFT = 0
RIGHT = 1

def exit_start_box(line, motors, speed=35, confirm_ms=120, timeout_ms=6000):

    start_time = ticks_ms()
    white_detected_since = None

    while ticks_diff(ticks_ms(), start_time) < timeout_ms:

        # drive straight forward 
        motors[LEFT].Forward(LEFT, speed)
        motors[RIGHT].Forward(RIGHT, speed)

        # read front sensors 
        left_front_on_line = line.leftOn.value()
        right_front_on_line = line.rightOn.value()

        both_front_sensors_on = (
            left_front_on_line and right_front_on_line
        )

        if both_front_sensors_on:
            if white_detected_since is None:
                white_detected_since = ticks_ms()
            elif ticks_diff(ticks_ms(), white_detected_since) >= confirm_ms:
                motors[LEFT].off()
                motors[RIGHT].off()
                sleep(0.05)
                return True
        else:
            white_detected_since = None

        sleep(0.01)

    # Timeout safety
    motors[LEFT].off()
    motors[RIGHT].off()
    return False


