from TrackerSensor import TrackerSensor
from Motor import Motor

class LineFollower():
    def __init__(self, tracker_sensors: list[TrackerSensor]):
        # Tracker sensors should be ordered from left to right
        self.__tracker_sensors = tracker_sensors
        self.__sensor_count = len(tracker_sensors)

    def get_line_pos(self) -> float | None:
        # Tracker sensor readings are either 0 or 1. Assuming tracker sensors are evenly spaced (with unit distance),
        # we can estimate the position of the line as the 'center of mass' of the readings.
        # 
        # The position will (hopefully) be a multiple of a half (if the thresholds are right), as 
        # a maximum of two sensors should read high.
        readings = [ts.read() for ts in self.__tracker_sensors]
        success = readings.count(True)
        if success:
            pos = sum([(i - 1.5) * readings[i] for i in range(self.__sensor_count)]) / success
            return pos
        return None

