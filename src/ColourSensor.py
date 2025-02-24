from machine import I2C, Pin
from tcs34725 import TCS34725

class ColourSensor():
    def __init__(self, sda_pin: int, scl_pin: int):
        i2c_bus = I2C(0, sda=Pin(sda_pin), scl=Pin(scl_pin))
        self.__tcs = TCS34725(i2c_bus)
    
    def read_rgbc(self) -> tuple[int, int, int, int]:
        return self.__tcs.read(raw=True)

    def read_temp_lux(self) -> tuple[int, int]:
        return self.__tcs.read(raw=False)
