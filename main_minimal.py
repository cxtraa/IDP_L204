import sys
sys.path.append("./src")

from constants import *
from Robot import Robot

def main():
    robot = Robot(graph=GRAPH)
    robot.forward()
    print("Complete")

if __name__ == "__main__":
    main()