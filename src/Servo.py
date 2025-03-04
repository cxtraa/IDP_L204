from machine import Pin, PWM


class Servo():
    MAX_DUTY = 7864
    MIN_DUTY = 1802
    DELTA_DUTY = MAX_DUTY - MIN_DUTY


    def __init__(self, pin_number: int):
        self.__pin_out = Pin(pin_number, Pin.OUT)
        self.__pwm = PWM(self.__pin_out)
        self.__pwm.freq(50)
        self.__pwm.duty_u16(0)
    

    def set_angle(self, angle: float) -> None:
        if angle > 270:
            angle = 270
        elif angle < 0:
            angle = 0
        
        duty_cycle = Servo.MIN_DUTY + Servo.DELTA_DUTY * angle / 270
        self.__pwm.duty_u16(int(duty_cycle))
