from utime import sleep, ticks_ms, ticks_diff

LEFT = 0
RIGHT = 1

def exit_start_box(line, motors, speed=35, confirm_ms=120, timeout_ms=6000):

    start_time = ticks_ms()
    white_detected_since = None

    while ticks_diff(ticks_ms(), start_time) < timeout_ms:

        # drive straight forward 
        motors[LEFT].Forward(LEFT, speed=speed)
        motors[RIGHT].Forward(RIGHT, speed=speed)

        # read front sensors 
        left_front_on_line = line.leftOn.value()
        right_front_on_line = line.rightOn.value()

         # both must be true (white)
        if left_front_on_line == 1 and right_front_on_line == 1:
            motors[LEFT].off()
            motors[RIGHT].off()
            sleep(0.05)
            return True

        sleep(0.01)

    # Timeout safety
    motors[LEFT].off()
    motors[RIGHT].off()
    return False


