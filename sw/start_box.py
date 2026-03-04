from utime import sleep, ticks_ms, ticks_diff

LEFT = 0
RIGHT = 1

def exit_start_box(line, motors, motorspeed=25, confirm_ms=120, timeout_ms=2000):

    start_time = ticks_ms()
    confirm_start = None

    while ticks_diff(ticks_ms(), start_time) < timeout_ms:

        # drive straight forward 
        motors[LEFT].Forward(LEFT, speed=motorspeed)
        motors[RIGHT].Forward(RIGHT, speed=motorspeed)

        # read front sensors 
        left_front_on_line = line.leftOn.value()
        right_front_on_line = line.rightOn.value()

         # both must be true (white)
        if left_front_on_line == 1 and right_front_on_line == 1:
            print("Both sensors on line.")
            print("speed: ", motorspeed)
            if confirm_start is None:
                print("Line detected, confirming...")
                confirm_start = ticks_ms()
            elif ticks_diff(ticks_ms(), confirm_start) > confirm_ms:
                print("Line confirmed. Exiting start box...")
                drive_start = ticks_ms()
                while ticks_diff(ticks_ms(), drive_start) < 800:
                    motors[LEFT].Forward(LEFT, speed=motorspeed)
                    motors[RIGHT].Forward(RIGHT, speed=motorspeed)
                    sleep(0.01)

                motors[LEFT].off()
                motors[RIGHT].off()
                print("straight stint complete, start line follow logic...")
                return True
        else:
            confirm_start = None

        sleep(0.01)

    # Timeout safety
    motors[LEFT].off()
    motors[RIGHT].off()
    print("Timeout: Failed to detect line within time limit.")
    return False


