
from reelsensor import ReelSensor

leftReelSDA=9
leftReelSCL=8
rightReelSDA=11
rightReelSCL=10

RIGHT = 1

def checktest():
    sensors = ReelSensor(8,9,10,11)
    something = sensors.check_reel_detected(RIGHT)



if __name__ == '__main__':
    checktest()