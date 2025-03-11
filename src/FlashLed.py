from machine import Pin


class FlashLed:
    def __init__(self, pin_number: int):
        self.__pin_out = Pin(pin_number, Pin.OUT)
        self.__pin_out.value(0)

    def flash(self):
        self.__pin_out.value(1)

    def off(self):
        self.__pin_out.value(0)

    def toggle(self):
        self.__pin_out.value(not self.__pin_out.value())
