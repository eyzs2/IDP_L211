from machine import Pin, I2C
from utime import sleep, ticks_diff, ticks_ms
from VL53L0X import VL53L0X
from pushbutton_logic import stop_function, StopRequested


from line_logic import LEFT, RIGHT, NO_TURN, T, FORWARD, REVERSE, FLIP

from grabber import TOP_RACK, BOTTOM_RACK, Grabber
THRESHOLD_DIST = 285  # Distance threshold in mm - adjust based on testing


class ReelSensor:
    def __init__(self, leftReelSDA, leftReelSCL, rightReelSDA, rightReelSCL):
        """
        Initialize reel sensors.
        """
        
        leftI2C = I2C(id=0, sda=Pin(leftReelSDA), scl=Pin(leftReelSCL)) # type: ignore # I2C0 on GP8 & GP9
        rightI2C = I2C(id=1, sda=Pin(rightReelSDA), scl=Pin(rightReelSCL)) # type: ignore #CHANGE TO ID 1 WHEN OTHER SENSOR CONNECTED
        print("i2c initialised?")
        self.distSensors = [VL53L0X(leftI2C),VL53L0X(rightI2C)]

        for sensor in self.distSensors:
            sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[0], 18)
            sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[1], 14)

    # function below returns average distance
    def get_distance(self, side):
        print("getting distance from side ", side)
        sensor = self.distSensors[side]
        # Start device
        sensor.start()
        distSamples = []
        # Read ten samples
        for _ in range(4):
            distance = sensor.read()
            print(f"Distance = {distance}mm")  # Check calibration!
            distSamples.append(distance)
            sleep(0.01)
        
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


    def grab(self, line, grabber:Grabber, side):

        print("Starting grab routine on side", side)

        # turn toward the reel side
        line.turnLogic(turnDirection=side)
        stop_function()

        grabber.grabber_align(BOTTOM_RACK)
        # move forward while still on line
        
        start_time = ticks_ms()

        while ticks_diff(ticks_ms(), start_time) < 700:
            line.lineFollow()

        for motor in line.motors:
            motor.off()

        grabber.pickup()

        print("pickup complete")

        start_time = ticks_ms()
        line.motors[LEFT].Reverse(LEFT, 60)
        line.motors[RIGHT].Reverse(RIGHT, 60)
        

        while ticks_diff(ticks_ms(), start_time) < 700:
            stop_function()

        grabber.grabber_align()
            

        line.motors[LEFT].off()
        line.motors[RIGHT].off()
        sleep(0.2) # might need to adjust

        line.turnLogic(turnDirection=FLIP) # turn back toward main line
        line.motors[LEFT].off()
        line.motors[RIGHT].off()    
        sleep(0.2) # might need to adjust

        print("turned back toward main line")

        
        print('ready for return route')
                    

            
