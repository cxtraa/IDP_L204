# DEMONSTRATION FILE THAT WAS PRE-LOADED ONTO RP

from machine import Pin , PWM
from utime import sleep

# Set up PWM Pin for servo control
servo1_pin = Pin(13)
servo1 = PWM(servo1_pin)

# Set Duty Cycle for Different Angles
max_duty = 7864
min_duty = 1802
half_duty = int(max_duty/2)

#Set PWM frequency
frequency = 50
servo1.freq(frequency)

dir1 = Pin(0,Pin.OUT)
pwm1 = PWM(Pin(1))
pwm1.freq(1000)

dir3 = Pin(4,Pin.OUT)
pwm3 = PWM(Pin(5))
pwm3.freq(1000)

def PushOut(duty):
    dir1.value(1)
    duty_16 = int((duty*65536)/100)
    pwm1.duty_u16(duty_16)
    
def PullIn(duty):
    dir1.value(0)
    duty_16 = int((duty*65536)/100)
    pwm1.duty_u16(duty_16)
    
def StopLinearAc():
    dir1.value(0)
    pwm1.duty_u16(0)

def RotateCW(duty):
    dir3.value(1)
    duty_16 = int((duty*65536)/100)
    pwm3.duty_u16(duty_16)

def RotateCCW(duty):
    dir3.value(0)
    duty_16 = int((duty*65536)/100)
    pwm3.duty_u16(duty_16)
    
def StopMotor():
    dir3.value(0)
    pwm3.duty_u16(0) 

while True:
    duty_cycle=(100)
    print (duty_cycle)
    PushOut(duty_cycle)
    sleep(8)
    PullIn(duty_cycle)
    sleep(8)
    StopLinearAc()
    sleep(1)
    RotateCW(duty_cycle)
    sleep(5)
    RotateCCW(duty_cycle)
    sleep(5)
    StopMotor()
    sleep(1)
    #Servo at 0 degrees
    servo1.duty_u16(min_duty)
    sleep(2)
    #Servo at 90 degrees
    servo1.duty_u16(half_duty)
    sleep(2)
    #Servo at 180 degrees
    servo1.duty_u16(max_duty)
    sleep(2) 

  