from utime import ticks_ms, ticks_diff, sleep_ms

START_EXIT = 0
RUN_MISSION = 1
RETURN_HOME = 2
STOPPED = 3

class BlackBoxController:
    """
    Uses existing LineSensor instance + existing motor list.
    Assumes:
      - White line => sensor value == 1
      - Black floor => sensor value == 0
    Start condition:
      - BOTH front sensors (leftOn, rightOn) read WHITE (1) for confirm_ms
    Home condition: (needs to occur only after the reel checking arrays are all full)
      - BOTH front sensors read BLACK (0) for confirm_ms
    """

    def __init__(self, line_sensor, motor,
                 exit_speed=35,
                 confirm_ms_start=100,
                 confirm_ms_home=500,
                 loop_delay_ms=20):
        self.line = line_sensor
        self.motor = motor

        self.exit_speed = exit_speed
        self.confirm_ms_start = confirm_ms_start
        self.confirm_ms_home = confirm_ms_home
        self.loop_delay_ms = loop_delay_ms

        self.state = START_EXIT
        self._start_since = None
        self._home_since = None

    def _front_left(self):
        return self.line.leftOn.value()

    def _front_right(self):
        return self.line.rightOn.value()

    def front_both_white(self):
        # both sensors detent we're on the white line
        return (self._front_left() == 1) and (self._front_right() == 1)

    def front_both_black(self):
        # both detect we're on black floor
        return (self._front_left() == 0) and (self._front_right() == 0)

    def _debounced(self, condition, since_attr, confirm_ms):
        now = ticks_ms()
        since = getattr(self, since_attr)

        if condition:
            if since is None:
                setattr(self, since_attr, now)
                return False
            return ticks_diff(now, since) >= confirm_ms
        else:
            setattr(self, since_attr, None)
            return False

    def step_start_exit(self):
        """
        Call repeatedly at boot.
        Drives forward until BOTH front sensors read white (debounced),
        then returns True meaning "start reel task now".
        """
        # drive straight out of the black box
        self.motor[0].Forward(self.exit_speed)
        self.motor[1].Forward(self.exit_speed)

        if self._debounced(self.front_both_white(), "_start_since", self.confirm_ms_start):
            # stop briefly to avoid overshoot
            self.motor[0].off()
            self.motor[1].off()
            sleep_ms(50)
            return True

        sleep_ms(self.loop_delay_ms)
        return False

