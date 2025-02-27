from machine import Pin
from time import ticks_ms
from constants import BUTTON_DEBOUNCE_TIME

class Button:
    def __init__(self, pin_number : int):
        self.__pin_in = Pin(pin_number, Pin.IN)
        self.last_debounce_time = 0.0
        self.state = 0 # low

    def read(self) -> int:
        return self.__pin_in.value()

    def update(self) -> None:
        if ticks_ms() - self.last_debounce_time > BUTTON_DEBOUNCE_TIME:
            self.state = self.read()