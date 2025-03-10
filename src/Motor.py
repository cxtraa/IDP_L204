from machine import Pin, PWM


class Motor:
    # Class for using the motor
    def __init__(self, motor_number):
        # Used for assigning the right motor number to the PWM and DIR
        motor_numbers = [(0, 1), (3, 2), (4, 5), (7, 6)]
        dir_pin, pwm_pin = motor_numbers[motor_number - 1]
        self.dir = Pin(dir_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)
        self.pwm.duty_u16(0)
    

    def off(self) -> None:
        self.pwm.duty_u16(0)
    

    def forward(self, speed : float) -> None:
        self.dir.value(0)
        self.set_duty_cycle(speed)
    

    def set_duty_cycle(self, percentage : float) -> None:
        if percentage > 100:
            percentage = 100
        elif percentage < 0:
            percentage = 0
        duty_cycle = int((2**16 - 1) * (percentage / 100)) # the board expects a 16-bit unsigned integer from 0 - (2^16-1) inclusive
        self.pwm.duty_u16(duty_cycle)


    def reverse(self, speed : float) -> None:
        self.dir.value(1)
        self.set_duty_cycle(speed)
