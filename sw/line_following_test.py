# line_follow_test_route.py
# Standalone Line-Following Test module (paste into a fresh .py file and run)
#
# UPDATED REQUIREMENTS (including your latest change):
#
# Sensors / meanings:
# - 4 sensors total:
#     FRONT pair (tracking): should be WHITE (1) during normal line following
#     REAR pair (decision): detects turn opportunities (junction/corner markers)
# - Sensor meaning: BLACK -> 0, WHITE -> 1
#
# Start behaviour:
# - Robot starts centred in BLACK start box (not touching white lines).
# - Drive forward until BOTH FRONT sensors are WHITE, then start the route logic.
#
# Turning behaviour (default, used for ALL turns except ONE special case):
# - When a REAR sensor detects a turn opportunity:
#     keep moving straight until BOTH FRONT sensors go BLACK,
#     then initiate a 90° turn in the chosen direction,
#     stop turning when BOTH FRONT sensors are WHITE again (line reacquired).
#
# Route script (as you specified earlier):
# 1) First executed turn: LEFT
# 2) Next four executed turns: RIGHT
# 3) Then: ignore next TWO LEFT opportunities
# 4) Then: on THIRD LEFT opportunity: turn LEFT and STOP (back at start box)
#
# SPECIAL CASE (your latest change):
# - The FINAL RIGHT turn (i.e., the 4th right in the “four rights” sequence) occurs at a place
#   where a white line continues straight, so the FRONT sensors stay WHITE and would never go BLACK.
# - Therefore:
#     After the THIRD RIGHT turn has been executed,
#     start counting RIGHT detections (rear-right opportunities),
#     and perform the FINAL RIGHT turn on the 7th RIGHT detection,
#     WITHOUT waiting for the front sensors to go black.
# - All other turns still use the “wait for front black” rule.

from machine import Pin, PWM
from utime import ticks_ms, ticks_diff, sleep_ms


# =============================
# CONFIG (EDIT THESE)
# =============================

# Motor pins (change to your real pins)
LEFT_MOTOR_DIR = 4
LEFT_MOTOR_PWM = 5
RIGHT_MOTOR_DIR = 6
RIGHT_MOTOR_PWM = 7

# Sensor pins (front = tracking, rear = decision)
FRONT_LEFT_PIN = 26
FRONT_RIGHT_PIN = 27
REAR_LEFT_PIN = 28
REAR_RIGHT_PIN = 29

# If you already use PULL_DOWN in your project, keep it. Otherwise try PULL_UP if readings are inverted.
SENSOR_PULL = "PULL_DOWN"  # "PULL_DOWN" or "PULL_UP"


# =============================
# Motor
# =============================
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


# =============================
# Sensors
# =============================
class Sensors:
    def __init__(self, front_left_pin: int, front_right_pin: int,
                 rear_left_pin: int, rear_right_pin: int,
                 pull: str = "PULL_DOWN"):
        p = Pin.PULL_DOWN if pull == "PULL_DOWN" else Pin.PULL_UP
        self.FL = Pin(front_left_pin, Pin.IN, p)
        self.FR = Pin(front_right_pin, Pin.IN, p)
        self.RL = Pin(rear_left_pin, Pin.IN, p)
        self.RR = Pin(rear_right_pin, Pin.IN, p)

    def front(self):
        return self.FL.value(), self.FR.value()

    def rear(self):
        return self.RL.value(), self.RR.value()


# =============================
# Controller
# =============================
FOLLOW = 0
WAIT_FRONT_BLACK = 1
TURNING = 2
STOPPED = 3

TURN_LEFT = -1
TURN_RIGHT = 1


class LineFollowingTestRoute:
    def __init__(self, left_motor: Motor, right_motor: Motor, sensors: Sensors):
        self.ML = left_motor
        self.MR = right_motor
        self.S = sensors

        # --- tuning (start with these, tweak on track) ---
        self.exit_speed = 35

        self.base_speed = 35
        self.steer_boost = 12
        self.steer_pulse_ms = 35

        self.loop_ms = 10

        self.rear_debounce_ms = 60
        self.front_black_confirm_ms = 70
        self.reacquire_confirm_ms = 80

        self.turn_speed = 45
        self.turn_step_ms = 35
        self.turn_max_ms = 1600

        self.stop_creep_ms = 450  # after final left, creep forward before stopping

        # --- state ---
        self.state = FOLLOW
        self.turn_dir = None
        self.turn_immediate = False  # <-- SPECIAL: for the final right turn only

        # -----------------------------
        # ROUTE SCRIPT STATE
        # -----------------------------
        self.turn_count = 0             # total turns executed successfully
        self.right_turns_executed = 0   # number of executed RIGHT turns
        self.left_opportunities = 0     # used after the 4-rights section
        self.stop_after_final_left = False

        # SPECIAL final-right logic
        self.count_right_detections = False
        self.right_detection_count = 0
        self.final_right_done = False

        # timers
        self._rear_since = None
        self._front_black_since = None
        self._reacquire_since = None
        self._turn_start_ms = None

    # -----------------
    # helpers
    # -----------------
    def stop(self):
        self.ML.off()
        self.MR.off()

    def drive_forward(self, speed=None):
        if speed is None:
            speed = self.base_speed
        self.ML.Forward(speed)
        self.MR.Forward(speed)

    @staticmethod
    def front_both_white(fl, fr):
        return (fl == 1) and (fr == 1)

    @staticmethod
    def front_both_black(fl, fr):
        return (fl == 0) and (fr == 0)

    def steer_left(self):
        self.ML.Forward(max(0, self.base_speed - self.steer_boost))
        self.MR.Forward(min(100, self.base_speed + self.steer_boost))
        sleep_ms(self.steer_pulse_ms)

    def steer_right(self):
        self.ML.Forward(min(100, self.base_speed + self.steer_boost))
        self.MR.Forward(max(0, self.base_speed - self.steer_boost))
        sleep_ms(self.steer_pulse_ms)

    def pivot_left_step(self):
        # pivot left: stop left motor, run right motor
        self.ML.off()
        self.MR.Forward(self.turn_speed)
        sleep_ms(self.turn_step_ms)

    def pivot_right_step(self):
        # pivot right: stop right motor, run left motor
        self.MR.off()
        self.ML.Forward(self.turn_speed)
        sleep_ms(self.turn_step_ms)

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

    # -----------------
    # tracking
    # -----------------
    def tracking_step(self):
        fl, fr = self.S.front()

        if self.front_both_white(fl, fr):
            self.drive_forward()
            return

        if fl == 0 and fr == 1:
            self.steer_right()
            return

        if fr == 0 and fl == 1:
            self.steer_left()
            return

        # both black
        self.drive_forward(max(0, self.base_speed - 10))

    # -----------------
    # rear opportunity detection
    # -----------------
    def detect_rear_turn(self):
        """
        Returns TURN_LEFT / TURN_RIGHT if a rear sensor indicates an opportunity (white=1).
        Debounced.
        If both rear are white, defaults to RIGHT.
        """
        rl, rr = self.S.rear()

        raw_dir = None
        if rl == 1 and rr == 0:
            raw_dir = TURN_LEFT
        elif rr == 1 and rl == 0:
            raw_dir = TURN_RIGHT
        elif rr == 1 and rl == 1:
            raw_dir = TURN_RIGHT

        if raw_dir is None:
            self._rear_since = None
            return None

        if self._rear_since is None:
            self._rear_since = ticks_ms()
            return None

        if ticks_diff(ticks_ms(), self._rear_since) >= self.rear_debounce_ms:
            return raw_dir

        return None

    # -----------------
    # route script decision
    # -----------------
    def choose_turn_action(self, detected_turn):
        """
        Returns a tuple:
          (chosen_dir, immediate)
        where:
          chosen_dir is TURN_LEFT / TURN_RIGHT / None (ignore)
          immediate is True only for the SPECIAL final-right case (turn without waiting front black)
        """

        # -------------------------
        # SPECIAL FINAL RIGHT LOGIC
        # -------------------------
        # After the THIRD RIGHT turn has been executed, start counting RIGHT detections.
        if (not self.final_right_done) and (self.right_turns_executed >= 3):
            self.count_right_detections = True

        if self.count_right_detections and (not self.final_right_done):
            # Only count RIGHT detections
            if detected_turn == TURN_RIGHT:
                self.right_detection_count += 1

                # On the 7th RIGHT detection -> perform the FINAL RIGHT turn immediately
                if self.right_detection_count == 7:
                    self.final_right_done = True
                    # This final right is the 4th right in your "four rights" section.
                    return (TURN_RIGHT, True)

            # Otherwise ignore all opportunities during this counting phase
            return (None, False)

        # -------------------------
        # NORMAL ROUTE SCRIPT
        # -------------------------
        # First executed turn: force LEFT
        if self.turn_count == 0:
            return (TURN_LEFT, False)

        # Next four executed turns: force RIGHT
        if 1 <= self.turn_count <= 4:
            return (TURN_RIGHT, False)

        # After 5 turns executed, ignore next two LEFT opportunities, then take the third LEFT and stop
        if detected_turn == TURN_LEFT:
            self.left_opportunities += 1

            if self.left_opportunities <= 2:
                return (None, False)

            if self.left_opportunities == 3:
                self.stop_after_final_left = True
                return (TURN_LEFT, False)

        # Ignore everything else
        return (None, False)

    # -----------------
    # start-box exit
    # -----------------
    def exit_start_box(self):
        self.drive_forward(self.exit_speed)
        fl, fr = self.S.front()
        return self.front_both_white(fl, fr)

    # -----------------
    # main loop step
    # -----------------
    def step(self):
        """
        Returns True once STOPPED.
        """
        if self.state == STOPPED:
            self.stop()
            return True

        fl, fr = self.S.front()

        # FOLLOW / decision
        if self.state == FOLLOW:
            self.tracking_step()

            detected = self.detect_rear_turn()
            if detected is not None:
                chosen, immediate = self.choose_turn_action(detected)

                # ignore opportunity
                if chosen is None:
                    sleep_ms(self.loop_ms)
                    return False

                # arm the turn
                self.turn_dir = chosen
                self.turn_immediate = immediate

                if self.turn_immediate:
                    # SPECIAL: start turning immediately, do NOT wait for front to go black
                    self.state = TURNING
                    self._turn_start_ms = ticks_ms()
                    self._reacquire_since = None
                else:
                    # Normal: wait for front both black before turning
                    self.state = WAIT_FRONT_BLACK
                    self._front_black_since = None

            sleep_ms(self.loop_ms)
            return False

        # Keep moving straight until BOTH front sensors are black (normal case)
        if self.state == WAIT_FRONT_BLACK:
            self.drive_forward(self.base_speed)

            if self._debounced_true(self.front_both_black(fl, fr), "_front_black_since", self.front_black_confirm_ms):
                self.state = TURNING
                self._turn_start_ms = ticks_ms()
                self._reacquire_since = None

            sleep_ms(self.loop_ms)
            return False

        # Turn until BOTH front sensors are white again
        if self.state == TURNING:
            # safety timeout
            if ticks_diff(ticks_ms(), self._turn_start_ms) >= self.turn_max_ms:
                self.stop()
                self.state = STOPPED
                return True

            if self.turn_dir == TURN_LEFT:
                self.pivot_left_step()
            else:
                self.pivot_right_step()

            fl, fr = self.S.front()
            if self._debounced_true(self.front_both_white(fl, fr), "_reacquire_since", self.reacquire_confirm_ms):
                # completed a turn
                self.turn_count += 1
                if self.turn_dir == TURN_RIGHT:
                    self.right_turns_executed += 1

                # reset turn flags
                self.turn_dir = None
                self.turn_immediate = False
                self._rear_since = None
                self.state = FOLLOW

                # If that was the final left, creep then stop
                if self.stop_after_final_left:
                    self.drive_forward(self.base_speed)
                    sleep_ms(self.stop_creep_ms)
                    self.stop()
                    self.state = STOPPED
                    return True

            sleep_ms(self.loop_ms)
            return False

        sleep_ms(self.loop_ms)
        return False


# =============================
# Main entrypoint
# =============================
def main():
    sensors = Sensors(
        FRONT_LEFT_PIN, FRONT_RIGHT_PIN,
        REAR_LEFT_PIN, REAR_RIGHT_PIN,
        pull=SENSOR_PULL
    )

    left_motor = Motor(LEFT_MOTOR_DIR, LEFT_MOTOR_PWM)
    right_motor = Motor(RIGHT_MOTOR_DIR, RIGHT_MOTOR_PWM)

    controller = LineFollowingTestRoute(left_motor, right_motor, sensors)

    # --- Start box exit ---
    while True:
        if controller.exit_start_box():
            controller.stop()
            sleep_ms(50)
            break
        sleep_ms(10)

    # --- Run route until it stops ---
    while True:
        if controller.step():
            break


if __name__ == "__main__":
    main()