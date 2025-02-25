from TrackerSensor import TrackerSensor
from Motor import Motor

class LineFollower:
    K_P = 2.5
    K_D = 0.02
    K_I = 12

    def __init__(self, tracker_sensors: list[TrackerSensor], sensor_pos: list[float]):
        # Tracker sensors should be ordered from left to right

        self.__tracker_sensors = [TrackerSensor()]
        self.__sensor_count = len(tracker_sensors)
        self.__sensor_pos = sensor_pos
        self.__last_error = self.get_line_pos()
        self.__integral_error = 0
    
    def get_ir_readings(self) -> list[int]:
        return [ts.read() for ts in self.__tracker_sensors]

    def get_line_pos(self) -> float | None:
        # Tracker sensor readings are either 0 or 1. Assuming tracker sensors are evenly spaced (with unit distance),
        # we can estimate the position of the line as the 'center of mass' of the readings.
        # 
        # The position will (hopefully) be a multiple of a half (if the thresholds are right), as 
        # a maximum of two sensors should read high.
        readings = self.get_ir_readings() # will look something like [0, 1, 1, 0]
        success = readings.count(True) # get "total mass"
        if success:
            pos = sum([self.__sensor_pos[i] * readings[i] for i in range(self.__sensor_count)]) / success
            return pos
        return None

    def get_power_error(self) -> float:
        # Returns how much extra power (+ve or -ve) needs to be delivered to right wheel than left to correct angle
        abs_error = self.get_line_pos()
        derivative_error = abs_error - self.__last_error
        self.__integral_error += abs_error

        # PID control
        power_error = abs_error * LineFollower.K_P + derivative_error * LineFollower.K_D + self.__integral_error * LineFollower.K_I
        return power_error
    
    def check_junction(self) -> bool:
        """
        Return true if the robot is currently at a junction, and false otherwise.
        """
        return self.get_ir_readings() != [0, 1, 1, 0]
        
    def reset(self):
        self.__last_error = self.get_line_pos()
        self.__integral_error = 0
