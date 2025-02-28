import sys
sys.path.append("/src")

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
        robot.navigate((-104, 162))
        robot.navigate((103,162))
        robot.navigate((0,-29))
    
    start_button = Button(12)
    while True:
        if start_button.read():
            start_button.irq(trigger=Button.IRQ_FALLING, handler=lambda p: quit())
            do_lap()
        sleep(0.1)

if __name__ == "__main__":
    main()
