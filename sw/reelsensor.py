from machine import Pin, I2C
from utime import sleep
from VL53L0X import VL53L0X

from line_logic import LEFT, RIGHT, NO_TURN, T, FORWARD, REVERSE

THRESHOLD_DIST = 300  # Distance threshold in mm - adjust based on testing


class ReelSensor:
    def __init__(self, leftReelSDA, leftReelSCL, rightReelSDA, rightReelSCL):
        """
        Initialize reel sensors.
        """
        
        # leftI2C = I2C(id=0, sda=Pin(leftReelSDA), scl=Pin(leftReelSCL)) # I2C0 on GP8 & GP9
        rightI2C = I2C(id=0, sda=Pin(rightReelSDA), scl=Pin(rightReelSCL)) #CHANGE TO ID 1 WHEN OTHER SENSOR CONNECTED
        print("i2c initialised?")
        self.distSensors = [VL53L0X(rightI2C)]
        # self.distSensors.insert(0, VL53L0X(leftI2C))
        
        # try:
        #     # Try to initialize as analog inputs (distance sensors)
        #     self.leftReelSensor = ADC(Pin(leftReelSensorPin))
        #     self.rightReelSensor = ADC(Pin(rightReelSensorPin))
        #     self.is_analog = True
        #     print("Reel sensors initialized as analog (ADC)")
        # except:
        #     # Fallback to digital pins
        #     self.leftReelSensor = Pin(leftReelSensorPin, Pin.IN)
        #     self.rightReelSensor = Pin(rightReelSensorPin, Pin.IN)
        #     print("Reel sensors initialized as digital pins")

        for Sensor in self.distSensors:
            Sensor.set_Vcsel_pulse_period(Sensor.vcsel_period_type[0], 18)
            Sensor.set_Vcsel_pulse_period(Sensor.vcsel_period_type[1], 14)


    def get_distance(self, side):
        print("getting distance from side ", side)
        side = 0 # TO REMOVE WHEN BOTH SENSORS CONNECTED
        sensor = self.distSensors[side]
        # Start device
        sensor.start()
        distSamples = []
        # Read ten samples
        for _ in range(4):
            distance = sensor.read()
            print(f"Distance = {distance}mm")  # Check calibration!
            distSamples.append(distance)
            sleep(0.1)
        
        # Stop device
        sensor.stop()
        distSamples.pop(0)
        averageDist = sum(distSamples) / len(distSamples)
        print("avg dist: ", averageDist)
        return averageDist
        

    def check_reel_detected(self, side):
        """
        Check if a reel is detected on the specified side (distance < THRESHOLD_DIST).
        
        Args:
            side: LEFT (0) or RIGHT (1)
        
        Returns:
            True if distance below threshold, False otherwise
        """
        distance = self.get_distance(side)
        if (distance < THRESHOLD_DIST):
            print("something there...")
        else:
            print('nothing burger')
        return distance < THRESHOLD_DIST


    def grab(self, line, side):
        line.turnLogic(turnDirection=side)
        while line.leftOn and line.rightOn:
            line.lineFollow()
        for motor in line.motors:
            motor.off()
        # TODO grabber logic

        for i in range(len(line.motors)):
            motor.Reverse(side=i, speed=10)

        while not (line.leftOn and line.rightOn):
            sleep(0.1)
        
        while not line.leftTurn and line.rightTurn:
            line.lineFollow(REVERSE)
            line.turnLogic(turnDirection=side)
            

            
