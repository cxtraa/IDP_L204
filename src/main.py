from Robot import Robot
from constants import *

def main():
    robot = Robot(
        graph=GRAPH,
        start_node=(0,-29),
        start_dir=0
    )

    robot.forward()
    print("Program finished.")

if __name__ == "__main__":
    main()