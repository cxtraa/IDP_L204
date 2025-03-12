import sys
sys.path.append("/src")

from Robot import Robot
from constants import *
from StateMachine import StateMachine
from time import sleep

def main():
    robot = Robot(graph=GRAPH, sensor_pos=SENSOR_POS)
    robot.servo.set_angle(30)
    sleep(1.0)
    print("Completed successfully")

if __name__ == "__main__":
    main()