from constants import GRAPH
from Robot import Robot
from StateMachine import Statemachine
import time

def main():
    robot = Robot(
        graph=GRAPH,
        start_node=(0,-29),
        start_dir=0
    )
    statemachine = Statemachine(robot)
    robot.navigate((-104, 0))

    statemachine = Statemachine(robot)

    # TODO: Robot continuously goes to each house. For each house, if there is a parcel, go to the depot, then go to the next house.
    # Otherwise, just go straight to the next house.

    # Define states
    NAVIGATE_TO_PICKUP = "NAVIGATE_TO_PICKUP"
    PICKUP_PACKAGE = "PICKUP_PACKAGE"
    NAVIGATE_TO_DEPOT = "NAVIGATE_TO_DEPOT"
    DROP_PACKAGE = "DROP_PACKAGE"
    RETURN_TO_START = "RETURN_TO_START"
    STOP_RUN = "STOP_RUN"

    # Initialize variables
    currentState = NAVIGATE_TO_PICKUP
    flag = 0

    # Main loop
    while currentState != STOP_RUN:
        if currentState == NAVIGATE_TO_PICKUP:
            statemachine.driveToPickup()
            currentState = PICKUP_PACKAGE

        elif currentState == PICKUP_PACKAGE:
            if statemachine.detectPackage():
                flag = 0
                statemachine.pickUpPackage()
                currentState = NAVIGATE_TO_DEPOT
            else:
                flag += 1
                if flag == 4:
                    currentState = RETURN_TO_START

        elif currentState == NAVIGATE_TO_DEPOT:
            colour = statemachine.getPackageColour()
            depot = statemachine.getDepotForColour(colour)
            statemachine.driveToDepot(depot)
            currentState = DROP_PACKAGE

        elif currentState == DROP_PACKAGE:
            statemachine.dropOffPackage()
            currentState = NAVIGATE_TO_PICKUP

        elif currentState == RETURN_TO_START:
            statemachine.driveToStart()
            currentState = STOP_RUN

        else:
            statemachine.stopMotors()
            break

    print("AGV has stopped. Task complete!")

if __name__ == "__main__":
    main()