from machine import I2C, Pin
from vl53l0x import VL53L0X
from constants import *


class TofSensor:
    def __init__(self, sda_pin: int, scl_pin: int):
        i2c_bus = I2C(1, sda=Pin(sda_pin), scl=Pin(scl_pin))
        self.__vl = VL53L0X(i2c_bus)
        self.__vl.set_measurement_timing_budget(40000)
        self.__vl.set_Vcsel_pulse_period(self.__vl.vcsel_period_type[0], 12)
        self.__vl.set_Vcsel_pulse_period(self.__vl.vcsel_period_type[1], 8)


    def read_distance(self) -> int:
        # Returns distance read by TOF sensor in mm
        return self.__vl.ping() - TOF_SENSOR_OFFSET
