import sys
sys.path.append("/src")

from machine import Pin
from Robot import Robot
from constants import *
from Servo import Servo

from time import sleep

def main():
#     robot = Robot(
#         graph=GRAPH,
#         start_node=(0,-29),
#         start_dir=0,
#         sensor_pos=[-5.25,-0.75,0.75,5.25]
#     )
    
    s = Servo(1)

    while True:
        s.set_angle(45)
        sleep(2)

    def procedure():
        robot.forward()
        robot.left()
        robot.forward()
        robot.right()
        robot.forward()
        robot.backwards_from_parcel()
        robot.reverse_left()
        robot.forward()
        
    def handle_stop(pin):
        quit()
    
    while True:
        if robot.start_button.pressed():
            # sleep(0.5)
            # start_button.irq(trigger=Pin.IRQ_FALLING, handler=handle_stop)
            procedure()
        sleep(0.1)

if __name__ == "__main__":
    main()
