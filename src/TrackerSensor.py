from machine import Pin

class TrackerSensor():
    def __init__(self, pin_number: int):
        self.__pin_in = Pin(pin_number, Pin.IN)
    
    def read(self) -> bool:
        return self.__pin_in.value()
