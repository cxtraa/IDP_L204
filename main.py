import sys
sys.path.append("/src")

from machine import Pin
from Button import Button
from Robot import Robot
from constants import *

from time import sleep

def main():
    robot = Robot(
        graph=GRAPH,
        start_node=(0,-29),
        start_dir=0,
        sensor_pos=[-5.25,-0.75,0.75,5.25]
    )

    def do_lap():
        robot.navigate((-27, 123))
        
    def handle_stop(pin):
        quit()
    
    while True:
        if start_button.pressed():
            sleep(0.5)
            #start_button.irq(trigger=Pin.IRQ_FALLING, handler=handle_stop)
            do_lap()
        sleep(0.1)

if __name__ == "__main__":
    main()
