from TrackerSensor import TrackerSensor
from constants import *

from math import copysign


class Control:
    def __init__(self, sensor_pos: list[float]):
        # Tracker sensors should be ordered from left to right
        self.__tracker_sensors = [
            TrackerSensor(IR1_PIN),
            TrackerSensor(IR2_PIN),
            TrackerSensor(IR3_PIN),
            TrackerSensor(IR4_PIN)
        ]
        self.__sensor_count = 4
        self.__sensor_pos = sensor_pos
        self.__last_error = self.get_line_pos() if self.get_line_pos() is not None else 0.0
        self.__integral_error = 0
    
    def get_ir_readings(self) -> list[int]:
        return [ts.read() for ts in self.__tracker_sensors]

    def get_line_pos(self) -> float | None:
        """
        The sensors return digital readings.
        No error: [0, 1, 1, 0]
        Line deviates to the right: [0, 0, 1, 1]
        Line deviates to the left: [1, 1, 0, 0]
        This function computes the center of mass of the readings, relative to the centre of the array.
        For example, [1, 1, 0, 0] has a COM of (-0.5*1 -1.5*1)/(1+1) = -1.
        """
        readings = self.get_ir_readings()
        sum_of_readings = sum(readings)
        if sum_of_readings:
            pos = sum([self.__sensor_pos[i] * readings[i] for i in range(self.__sensor_count)]) / sum_of_readings
            return pos
        return None

    def get_pid_error(self) -> float:
        """
        Compute the PID error of the IR sensor position.
        If the `power_error` computed is > 0, it means that line deviates to the right.
        Hence, the power of the left wheel should be increased by `power_error`, and right wheel decreased by same amount.
        """
        abs_error = self.get_line_pos()

        if abs_error is None: # No line detected
            abs_error = copysign(3.0, self.__last_error) # Assume the line is slightly closer to middle than it was last time
        
        derivative_error = abs_error - self.__last_error
        self.__last_error = abs_error
        self.__integral_error += abs_error
        pid_error = abs_error * K_P + derivative_error * K_D + self.__integral_error * K_I
        return pid_error
    
    def at_junction(self) -> bool:
        """
        Return true if the robot is currently at a junction, and false otherwise.

        Since the left most sensor and right most sensor are very far out, if either of these go to 1 the robot is almost certainly at a junction.
        """
        readings = self.get_ir_readings()
        return (readings[0] == 1 or readings[3] == 1) and (readings.count(True) >= 2)
        
    def reset(self):
        self.__last_error = self.get_line_pos()
        self.__integral_error = 0
