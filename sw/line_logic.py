from machine import Pin

def onLine():
    leftonLinePin = 26
    rightonLinePin = 27
    leftonLine = Pin(leftonLinePin, Pin.IN, Pin.PULL_DOWN)
    leftonLine.irq(handler=offLine)
    rightonLine = Pin(rightonLinePin, Pin.IN, Pin.PULL_DOWN)
    rightonLine.irq(handler=offLine)

def offLine(p):
    value = p.value()
    print(f"Input changed, value={value}")


    