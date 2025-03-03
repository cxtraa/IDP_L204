from constants import *
from PathFinder import PathFinder
from Control import Control
from Motor import Motor
from time import sleep, ticks_ms
from Robot import Robot

class StateMachine:
    def __init__(self):
        self.robot = Robot(
            graph=GRAPH,
            start_node=START_POINT,
            start_dir=0,
            sensor_pos=SENSOR_POS
        )
        self.t = 0 # represents current time
        self.i = 0  # Represents which pickup point we are at (0, 1, 2, 3)
        self.num_empty_parcel = 0
        self.should_end = False

    def update(self) -> None:
        self.robot.navigate(PICKUP_POINTS[self.i])
        dest_node = self.robot.pickup_parcel(next_pickup_location=PICKUP_POINTS[(self.i + 1) % 4])
        if dest_node == DEPOT_RED_YELLOW or dest_node == DEPOT_BLUE_GREEN:
            self.robot.navigate(dest_node)
            self.robot.depot_procedure(dest_node)
        else:
            self.num_empty_parcel += 1
            if self.num_empty_parcel == 4:
                self.should_end = True
        self.i = (self.i + 1) % 4

    def back_to_start(self) -> None:
        self.robot.navigate(START_POINT)  

    def stop(self) -> None:
        self.robot.left_motor.off()
        self.robot.right_motor.off()