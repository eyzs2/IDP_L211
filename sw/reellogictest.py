
from reelsensor import ReelSensor

leftReelSDA=0
leftReelSCL=0
rightReelSDA=8
rightReelSCL=9

RIGHT = 1

def checktest():
    sensors = ReelSensor(0,0,8,9)
    something = sensors.check_reel_detected(RIGHT)



if __name__ == '__main__':
    checktest()