# line_follow_turns.py
# MicroPython (Raspberry Pi Pico / PicoBot)
#
# Behaviour (exactly as you described):
# - 4 sensors: 2 FRONT (track the line), 2 REAR (turn detectors, wider)
# - While following: front sensors aim to stay on WHITE line (white=1)
# - When ONE rear sensor detects a turn marker:
#     1) Keep driving straight forward
#     2) Wait until BOTH front sensors go BLACK (0)  -> end of the straight / corner entry
#     3) Initiate a 90° turn in the direction of the rear sensor that triggered
#     4) Stop turning when BOTH front sensors see WHITE again (line reacquired)
#
# Notes:
# - Assumes sensor spec: Black=0, White=1 (common for these modules).
# - "90°" is achieved by turning until line reacquired, with a max timeout safety.

from machine import Pin, PWM
from utime import ticks_ms, ticks_diff, sleep_ms


# -----------------------------
# Motor
# -----------------------------
class Motor:
    def __init__(self, dirPin: int, PWMPin: int, pwm_freq: int = 1000):
        self.dir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(pwm_freq)
        self.pwm.duty_u16(0)

    def off(self):
        self.pwm.duty_u16(0)

    def Forward(self, speed: int = 35):
        speed = max(0, min(100, speed))
        self.dir.value(0)  # forward
        self.pwm.duty_u16(int(65535 * speed / 100))

    def Reverse(self, speed: int = 35):
        speed = max(0, min(100, speed))
        self.dir.value(1)  # reverse
        self.pwm.duty_u16(int(65535 * speed / 100))


# -----------------------------
# Sensors (2 front, 2 rear)
# -----------------------------
class Sensors:
    def __init__(self, front_left_pin: int, front_right_pin: int,
                 rear_left_pin: int, rear_right_pin: int,
                 pull: str = "PULL_DOWN"):
        # If your sensor outputs are strong TTL (typical), pull doesn't matter much.
        # Keep it consistent with how you already wired your project.
        p = Pin.PULL_DOWN if pull == "PULL_DOWN" else Pin.PULL_UP

        self.FL = Pin(front_left_pin, Pin.IN, p)
        self.FR = Pin(front_right_pin, Pin.IN, p)
        self.RL = Pin(rear_left_pin, Pin.IN, p)
        self.RR = Pin(rear_right_pin, Pin.IN, p)

    def front(self):
        return self.FL.value(), self.FR.value()

    def rear(self):
        return self.RL.value(), self.RR.value()


# -----------------------------
# Controller
# -----------------------------
FOLLOW = 0
WAIT_FRONT_BLACK = 1
TURNING = 2

TURN_LEFT = -1
TURN_RIGHT = 1


class LineFollowTurnController:
    def __init__(self, left_motor: Motor, right_motor: Motor, sensors: Sensors,
                 # speeds
                 base_speed: int = 35,
                 steer_boost: int = 12,
                 # timing
                 loop_ms: int = 10,
                 rear_debounce_ms: int = 50,
                 front_black_confirm_ms: int = 60,
                 reacquire_confirm_ms: int = 80,
                 # turning
                 turn_speed: int = 40,
                 turn_max_ms: int = 1400,
                 # optional: short settle pauses
                 settle_ms: int = 20):
        self.ML = left_motor
        self.MR = right_motor
        self.S = sensors

        self.base_speed = base_speed
        self.steer_boost = steer_boost

        self.loop_ms = loop_ms
        self.rear_debounce_ms = rear_debounce_ms
        self.front_black_confirm_ms = front_black_confirm_ms
        self.reacquire_confirm_ms = reacquire_confirm_ms

        self.turn_speed = turn_speed
        self.turn_max_ms = turn_max_ms
        self.settle_ms = settle_ms

        self.state = FOLLOW
        self.turn_dir = None  # TURN_LEFT or TURN_RIGHT

        # debounce timers
        self._rear_since = None
        self._front_black_since = None
        self._reacquire_since = None
        self._turn_start_ms = None

    # ---- helpers ----
    @staticmethod
    def front_both_black(fl, fr):
        return (fl == 0) and (fr == 0)

    @staticmethod
    def front_both_white(fl, fr):
        return (fl == 1) and (fr == 1)

    def stop(self):
        self.ML.off()
        self.MR.off()

    def drive_forward(self, speed=None):
        if speed is None:
            speed = self.base_speed
        self.ML.Forward(speed)
        self.MR.Forward(speed)

    def steer_left(self):
        # turn left gently while moving: slow left, speed up right
        self.ML.Forward(max(0, self.base_speed - self.steer_boost))
        self.MR.Forward(min(100, self.base_speed + self.steer_boost))

    def steer_right(self):
        # turn right gently while moving: speed up left, slow right
        self.ML.Forward(min(100, self.base_speed + self.steer_boost))
        self.MR.Forward(max(0, self.base_speed - self.steer_boost))

    def pivot_left(self):
        # in-place-ish pivot left
        self.ML.off()
        self.MR.Forward(self.turn_speed)

    def pivot_right(self):
        # in-place-ish pivot right
        self.MR.off()
        self.ML.Forward(self.turn_speed)

    def _debounced_true(self, cond: bool, since_attr: str, ms: int) -> bool:
        now = ticks_ms()
        since = getattr(self, since_attr)
        if cond:
            if since is None:
                setattr(self, since_attr, now)
                return False
            return ticks_diff(now, since) >= ms
        setattr(self, since_attr, None)
        return False

    # ---- core behaviour ----
    def follow_step(self):
        """
        Simple front-based tracking:
        - Ideal: both front sensors white -> straight
        - If one front goes black -> steer towards the white line
        """
        fl, fr = self.S.front()

        if self.front_both_white(fl, fr):
            self.drive_forward()
            return

        if fl == 0 and fr == 1:
            # left on black => line is more to the right => steer right
            self.steer_right()
            return

        if fr == 0 and fl == 1:
            # right on black => line is more to the left => steer left
            self.steer_left()
            return

        # both black: leave handling to the turn logic state machine
        self.drive_forward(max(0, self.base_speed - 10))

    def detect_rear_turn(self) -> int | None:
        """
        Returns TURN_LEFT or TURN_RIGHT if a rear sensor indicates a turn marker (white=1),
        with debounce so it doesn't flicker-trigger.
        Priority: if both rear see white at once, choose right by default (can change).
        """
        rl, rr = self.S.rear()

        # raw detection: rear sensor ON white marker
        raw_dir = None
        if rl == 1 and rr == 0:
            raw_dir = TURN_LEFT
        elif rr == 1 and rl == 0:
            raw_dir = TURN_RIGHT
        elif rr == 1 and rl == 1:
            # both triggered: pick a consistent default (right) or change to left
            raw_dir = TURN_RIGHT

        if raw_dir is None:
            self._rear_since = None
            return None

        # debounce "a turn direction is present"
        if self._rear_since is None:
            self._rear_since = ticks_ms()
            return None

        if ticks_diff(ticks_ms(), self._rear_since) >= self.rear_debounce_ms:
            return raw_dir

        return None

    def step(self):
        """
        Call repeatedly in main loop.
        """
        fl, fr = self.S.front()

        # -----------------
        # FOLLOW state
        # -----------------
        if self.state == FOLLOW:
            # Run normal tracking
            self.follow_step()

            # If a rear sensor sees a turn marker, arm a turn
            turn = self.detect_rear_turn()
            if turn is not None:
                self.turn_dir = turn
                self.state = WAIT_FRONT_BLACK
                self._front_black_since = None
                sleep_ms(self.settle_ms)

            sleep_ms(self.loop_ms)
            return

        # -----------------
        # WAIT_FRONT_BLACK state
        # Keep going straight until BOTH front sensors go black
        # -----------------
        if self.state == WAIT_FRONT_BLACK:
            # Keep moving straight (as you requested)
            self.drive_forward(self.base_speed)

            if self._debounced_true(self.front_both_black(fl, fr), "_front_black_since", self.front_black_confirm_ms):
                # start turning
                self.state = TURNING
                self._turn_start_ms = ticks_ms()
                self._reacquire_since = None
                sleep_ms(self.settle_ms)

            sleep_ms(self.loop_ms)
            return

        # -----------------
        # TURNING state
        # Pivot until BOTH front sensors are white again (line reacquired)
        # -----------------
        if self.state == TURNING:
            # safety timeout
            if ticks_diff(ticks_ms(), self._turn_start_ms) >= self.turn_max_ms:
                self.stop()
                # fall back to FOLLOW (or stop) – here we fall back to FOLLOW
                self.state = FOLLOW
                self.turn_dir = None
                sleep_ms(50)
                return

            # perform turn
            if self.turn_dir == TURN_LEFT:
                self.pivot_left()
            else:
                self.pivot_right()

            # stop turning once the front pair are back on the white line
            if self._debounced_true(self.front_both_white(fl, fr), "_reacquire_since", self.reacquire_confirm_ms):
                self.state = FOLLOW
                self.turn_dir = None
                self._rear_since = None
                sleep_ms(self.settle_ms)

            sleep_ms(self.loop_ms)
            return


# -----------------------------
# Example main runner
# -----------------------------
def run():
    # >>> CHANGE THESE PINS TO YOUR ACTUAL WIRING <<<
    # Motors
    LEFT_DIR = 4
    LEFT_PWM = 5
    RIGHT_DIR = 6
    RIGHT_PWM = 7

    # Sensors (front two: tracking; rear two: junction/corner markers)
    FRONT_LEFT = 26
    FRONT_RIGHT = 27
    REAR_LEFT = 28
    REAR_RIGHT = 29

    # If your existing project uses PULL_DOWN for these pins, keep it.
    sensors = Sensors(FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT, pull="PULL_DOWN")

    left_motor = Motor(LEFT_DIR, LEFT_PWM)
    right_motor = Motor(RIGHT_DIR, RIGHT_PWM)

    controller = LineFollowTurnController(
        left_motor, right_motor, sensors,
        base_speed=35,
        steer_boost=12,
        rear_debounce_ms=60,
        front_black_confirm_ms=70,
        reacquire_confirm_ms=80,
        turn_speed=45,
        turn_max_ms=1400
    )

    while True:
        controller.step()


if __name__ == "__main__":
    run()