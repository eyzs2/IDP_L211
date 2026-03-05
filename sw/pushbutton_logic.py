from utime import ticks_ms, ticks_diff
import micropython

class ButtonEdge:

    def __init__(self, pin, debounce_ms=200):
        self.pin = pin
        self.debounce_ms = debounce_ms
        self._last_state = 0
        self._last_press_ms = 0

    def pressed(self) -> bool:
        now = ticks_ms()
        state = self.pin.value()

        fired = False
        if state == 1 and self._last_state == 0:
            if ticks_diff(now, self._last_press_ms) > self.debounce_ms:
                self._last_press_ms = now
                fired = True

        self._last_state = state
        return fired
    
class StopRequested(Exception):
    """Raised via micropython.schedule from button IRQ to abort execution."""
    pass

def _sched_raise(_arg):
    # runs in main context (scheduled), raising will unwind the running code
    raise StopRequested()

def request_stop_irq(pin=None):
    # call from IRQ handler: schedule a raise in main context
    micropython.schedule(_sched_raise, 0)
    