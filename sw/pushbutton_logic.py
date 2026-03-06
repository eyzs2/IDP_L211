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
    
class StopRequested(BaseException):
    """Raised via micropython.schedule from button IRQ to abort execution."""
    pass

# IRQ-safe: schedule a small setter in main context
_STOP_REQUESTED = False

def _sched_set_flag(_arg):
    global _STOP_REQUESTED
    _STOP_REQUESTED = True

def request_stop_irq(pin=None):
    # call from IRQ handler: schedule a raise in main context
    micropython.schedule(_sched_set_flag, 0)

def stop_requested() -> bool:
    return _STOP_REQUESTED

def clear_stop():
    global _STOP_REQUESTED
    _STOP_REQUESTED = False

def stop_function():
    if _STOP_REQUESTED:
        raise StopRequested()
    