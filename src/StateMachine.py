from constants import *
from PathFinder import PathFinder
from Control import Control
from Motor import Motor
from time import sleep
from Robot import Robot

class Statemachine:
    
    def __init__(self, robot):
        self.robot = robot
        self.pickup_point_init = 0  # Initialize pickup point counter
    
    def driveToPickup(self):
        pickup_points = [pickup_1, pickup_2, pickup_3, pickup_4]
        self.robot.navigate(pickup_points[self.pickup_point_init])
        self.pickup_point_init = (1 + self.pickup_point_init) % 4
        print("Driving to pickup point...")
        sleep(1)

    def detectPackage(self):
        print("Checking for package...")
        return True

    def pickUpPackage(self):
        print("Picking up package...")
        sleep(1)

    def getPackageColour(self):
        return "blue"

    def getDepotForColour(self, colour):
        return 1 if colour in ["blue", "green"] else 2

    def driveToDepot(self, depot):
        print(f"Driving to Depot {depot}...")
        self.robot.navigate(depot)
        sleep(1)

    def dropOffPackage(self):
        print("Dropping off package...")
        sleep(1)

    def driveToStart(self):
        print("Returning to start box...")
        self.robot.navigate(start_point)
        sleep(1)

    def stopMotors(self):
        print("Motors stopped.")
        self.robot.left_motor.off()
        self.robot.right_motor.off()



    