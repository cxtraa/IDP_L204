from machine import I2C, Pin
from tcs34725 import TCS34725
from constants import *

class ColourSensor():
    def __init__(self, sda_pin: int, scl_pin: int):
        i2c_bus = I2C(0, sda=Pin(sda_pin), scl=Pin(scl_pin))
        self.__tcs = TCS34725(i2c_bus)
    
    def read_rgbc(self) -> tuple[int, int, int, int]:
        return self.__tcs.read(raw=True)

    def read_temp_lux(self) -> tuple[int, int]:
        return self.__tcs.read(raw=False)

    def read_colour(self) -> int:
        temp_lux = self.read_temp_lux()
        if temp_lux in COLOUR_READINGS:
            return COLOUR_READINGS[temp_lux]
        else:
            return min(COLOUR_READINGS, key=lambda x: ColourSensor.colour_error(temp_lux, x))

    @staticmethod
    def colour_error(temp_lux_in, temp_lux_ref) -> int:
        return sum(abs(temp_lux_in[i] - temp_lux_ref[i]) for i in range(2))
