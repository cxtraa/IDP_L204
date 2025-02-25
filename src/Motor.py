from machine import Pin, PWM
from time import sleep

# Class for using the motor

class Motor:
    def __init__(self, motor_number):
        # Used for assigning the right motor number to the PWM and DIR
        motor_numbers = [(0, 1), (3, 2), (4, 5), (7, 6)]
        dir_pin, pwm_pin = motor_numbers[motor_number - 1]
        self.m1Dir = Pin(dir_pin, Pin.OUT)
        self.pwm1 = PWM(Pin(pwm_pin))
        self.pwm1.freq(1000)
        self.pwm1.duty_u16(0)
    
    def off(self) -> None:
        self.pwm1.duty_u16(0)
    
    def forward(self) -> None:
        self.m1Dir.value(0)
        self.set_duty_cycle(100)
    
    def set_duty_cycle(self, percentage : float) -> int:
        duty_cycle = int((2**16 - 1) * percentage) # the board expects a 16 bit unsigned integer from 0 - (2^16-1) inclusive
        self.pwm1.duty_u16(duty_cycle)

    def reverse(self) -> None:
        self.m1Dir.value(1)
        self.set_duty_cycle(30)