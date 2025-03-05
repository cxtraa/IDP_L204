from constants import *
from Robot import Robot


class StateMachine:
    def __init__(self):
        self.robot = Robot(
            graph=GRAPH,
            sensor_pos=SENSOR_POS,
            start_node=START_POINT,
            start_dir=0
        )
        self.t = 0 # represents current time
        self.i = 0  # Represents which pickup point we are at (0, 1, 2, 3)
        self.num_empty_parcel = 0
        self.should_end = False

    def update(self) -> None:
        self.check_time_sufficient(PICKUP_POINTS[self.i])
        self.robot.navigate(PICKUP_POINTS[self.i])

        dest_node = self.robot.pickup_parcel(next_pickup_location=PICKUP_POINTS[(self.i + 1) % 4])
        if dest_node == DEPOT_RED_YELLOW or dest_node == DEPOT_BLUE_GREEN:
            self.check_time_sufficient(dest_node)
            self.robot.navigate(dest_node)
            self.robot.depot_procedure(dest_node)
        else:
            self.num_empty_parcel += 1
            if self.num_empty_parcel == 4:
                self.should_end = True
        self.i = (self.i + 1) % 4

    def check_time_sufficient(self, dest : tuple[int, int]) -> None:
        """
        Send the robot back to the beginning if there is not enough time.
        """
        time = self.robot.time_for_path(dest)
        if TOTAL_ALLOWED_TIME - self.t < time:
            self.back_to_start()
            return
        self.t += time

    def back_to_start(self) -> None:
        self.robot.navigate(START_POINT)  
        self.stop()

    def stop(self) -> None:
        self.robot.left_motor.off()
        self.robot.right_motor.off()