from machine import Pin, ADC
from utime import sleep

from line_logic import LEFT, RIGHT, NO_TURN, T

THRESHOLD_DIST = 300  # Distance threshold in mm - adjust based on testing


class ReelSensor:
    def __init__(self, leftReelSensorPin, rightReelSensorPin):
        """
        Initialize reel sensors.
        Will attempt to use ADC for analog distance sensors, fallback to digital pins.
        """
        self.is_analog = False
        
        try:
            # Try to initialize as analog inputs (distance sensors)
            self.leftReelSensor = ADC(Pin(leftReelSensorPin))
            self.rightReelSensor = ADC(Pin(rightReelSensorPin))
            self.is_analog = True
            print("Reel sensors initialized as analog (ADC)")
        except:
            # Fallback to digital pins
            self.leftReelSensor = Pin(leftReelSensorPin, Pin.IN)
            self.rightReelSensor = Pin(rightReelSensorPin, Pin.IN)
            print("Reel sensors initialized as digital pins")
        
        self.reelSensors = [self.leftReelSensor, self.rightReelSensor]

    def get_distance(self, side):
        """
        Get distance reading from specified reel sensor.
        
        Args:
            side: LEFT (0) or RIGHT (1)
        
        Returns:
            Distance value (0-65535 for ADC, 0-1 for digital)
        """
        try:
            if self.is_analog:
                return self.reelSensors[side].read_u16()
            else:
                return self.reelSensors[side].value()
        except Exception as e:
            print(f"Sensor read error: {e}")
            return 999999  # Return high value to avoid false positives

    def check_reel_detected(self, side):
        """
        Check if a reel is detected on the specified side (distance < THRESHOLD_DIST).
        
        Args:
            side: LEFT (0) or RIGHT (1)
        
        Returns:
            True if distance below threshold, False otherwise
        """
        distance = self.get_distance(side)
        return distance < THRESHOLD_DIST
            
