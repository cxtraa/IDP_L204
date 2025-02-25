
from constants import GRAPH
from Robot import Robot
import time

def main():
    robot = Robot(
        graph=GRAPH,
        start_node=(0,-29),
        start_dir=0
    )

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

    # Dummy functions to simulate AGV actions
    def driveToPickup():
        print("Driving to pickup point...")
        time.sleep(1)

    def detectPackage():
        print("Checking for package...")
        return True

    def pickUpPackage():
        print("Picking up package...")
        time.sleep(1)

    def getPackageColour():
        return "blue"

    def getDepotForColour(colour):
        return 1 if colour in ["blue", "green"] else 2

    def driveToDepot(depot):
        print(f"Driving to Depot {depot}...")
        time.sleep(1)

    def dropOffPackage():
        print("Dropping off package...")
        time.sleep(1)

    def driveToStart():
        print("Returning to start box...")
        time.sleep(1)

    def stopMotors():
        print("Motors stopped.")

    # Main loop
    while currentState != STOP_RUN:
        if currentState == NAVIGATE_TO_PICKUP:
            driveToPickup()
            currentState = PICKUP_PACKAGE

        elif currentState == PICKUP_PACKAGE:
            if detectPackage():
                flag = 0
                pickUpPackage()
                currentState = NAVIGATE_TO_DEPOT
            else:
                flag += 1
                if flag == 4:
                    currentState = RETURN_TO_START

        elif currentState == NAVIGATE_TO_DEPOT:
            colour = getPackageColour()
            depot = getDepotForColour(colour)
            driveToDepot(depot)
            currentState = DROP_PACKAGE

        elif currentState == DROP_PACKAGE:
            dropOffPackage()
            currentState = NAVIGATE_TO_PICKUP

        elif currentState == RETURN_TO_START:
            driveToStart()
            currentState = STOP_RUN

        else:
            stopMotors()
            break

    print("AGV has stopped. Task complete!")

if __name__ == "__main__":
    main()