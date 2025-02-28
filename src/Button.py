from machine import Pin
from time import ticks_ms
from constants import BUTTON_DEBOUNCE_TIME

class Button:
    def __init__(self, pin_number : int):
        self.__pin_in = Pin(pin_number, Pin.IN)
        self.last_debounce_time = 0.0
        self.state = 0 # low

    def __read(self) -> int:
        return self.__pin_in.value()

    def __update(self) -> None:
        if ticks_ms() - self.last_debounce_time > BUTTON_DEBOUNCE_TIME:
            self.state = self.__read()
    
    def pressed(self) -> bool:
        self.__update()
        if self.state == 1:
            return True
        return False
    
    def irq(self, trigger, handler) -> None:
        self.__pin_in.irq(trigger, handler)