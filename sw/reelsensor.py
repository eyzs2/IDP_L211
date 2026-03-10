from machine import Pin, I2C
from utime import sleep
from VL53L0X import VL53L0X
from pushbutton_logic import stop_function

from line_logic import LEFT, RIGHT, NO_TURN, T, FORWARD, REVERSE

THRESHOLD_DIST = 300  # Distance threshold in mm - adjust based on testing


class ReelSensor:
    def __init__(self, leftReelSDA, leftReelSCL, rightReelSDA, rightReelSCL):
        """
        Initialize reel sensors.
        """
        
        # leftI2C = I2C(id=0, sda=Pin(leftReelSDA), scl=Pin(leftReelSCL)) # I2C0 on GP8 & GP9
        rightI2C = I2C(id=0, sda=Pin(rightReelSDA), scl=Pin(rightReelSCL), freq=400000) # type: ignore

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

        for sensor in self.distSensors:
            sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[0], 18)
            sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[1], 14)

    # function below returns average distance
    def get_distance(self, side):
        print("getting distance from side ", side)
        sensor = self.distSensors[0]
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
        if len(distSamples) > 1:
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
        if distance < THRESHOLD_DIST:
            print("something there...")
            return True
        else:
            print("no reel detected")
            return False


    def grab(self, line, side):

        """
        Temporary placeholder grab routine.

        Current behaviour:
        1. Turn toward requested side
        2. Move forward until front line is lost
        3. Stop for pickup
        4. Reverse until front line is found again
        5. Stop

        Replace the section later with actual grabber actuation.
        """

        print("Starting grab routine on side", side)

        # turn toward the reel side
        line.turnLogic(turnDirection=side)
        # move forward while still on line
        while line.leftOn.value() and line.rightOn.value():
            line.lineFollow()
        for motor in line.motors:
            motor.off()

        # TODO grabber logic
        print("Pretending to grab...")
        sleep(0.5)

        for i in range(len(line.motors)):
            line.motors[i].Reverse(side=i, speed=20)

        while not (line.leftTurn.value() or line.rightTurn.value()):
            line.lineFollow(REVERSE)

        for motor in line.motors:
            motor.off()
                    

            
