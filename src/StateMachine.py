from constants import *
from PathFinder import PathFinder
from Control import Control
from Motor import Motor
from time import sleep
from Robot import Robot

class StateMachine:
    
    def __init__(self):
        self.robot = Robot(
            graph=GRAPH,
            start_node=(0, -29),
            start_dir=0,
            sensor_pos=SENSOR_POS
        )
        self.i = 0  # Represents which pickup point we are at (0, 1, 2, 3)
        self.num_empty_parcel = 0
        self.should_end = False

    def update(self) -> None:
        self.robot.navigate(PICKUP_POINTS[self.i])
        dest_depot = self.robot.pickup_parcel()
        if dest_depot is not None:
            self.robot.navigate(dest_depot)
            self.robot.depot_procedure()
        else:
            self.num_empty_parcel += 1
            if self.num_empty_parcel == 4:
                self.should_end = True
        self.i = (self.i + 1) % 4   

    def stop(self) -> None:
        self.robot.left_motor.off()
        self.robot.right_motor.off()
