from constants import GRAPH
from Robot import Robot

def main():
    robot = Robot(
        graph=GRAPH,
        start_node=(0,-29),
        start_dir=0
    )
    robot.navigate((-104, 0))

    # TODO: Robot continuously goes to each house. For each house, if there is a parcel, go to the depot, then go to the next house.
    # Otherwise, just go straight to the next house.

if __name__ == "__main__":
    main()