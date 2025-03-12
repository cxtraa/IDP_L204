from constants import *
from Robot import Robot

from time import ticks_ms, ticks_diff


class StateMachine:
    def __init__(self):
        self.robot = Robot(
            graph=GRAPH,
            sensor_pos=SENSOR_POS,
            start_node=START_POINT,
            start_dir=0
        )
        self.time_start = ticks_ms() # represents current time
        self.i = 0  # Represents which pickup point we are at (0, 1, 2, 3)
        self.num_empty_parcel = 0
        self.should_end = False

    def start_procedure(self) -> None:
        while not self.robot.control.at_junction():
            self.robot.left_motor.forward(ROBOT_SPEED_LINE)
            self.robot.right_motor.forward(ROBOT_SPEED_LINE)        
        
        self.robot.left_motor.off()
        self.robot.right_motor.off()

    def update(self) -> None:
        # - Check if we are safe to try to pick up a parcel
        # - i.e. check time taken for journey to the next pickup point then to the starting node (home) is less than
        #   the amount of time left before the end of 5 minutes
        #   - If we do, navigate to this pickup point and try to pick up a parcel
        #       - If we find a parcel, check the colour and verify that we have enough time to deliver it correctly and get home
        #           - If we do, go to delivery point and do depot_procedure
        #           - Else if we don't, then we should still have time to get home, so do that
        #       - Else if there is no parcel, add one to empty_parcels and increment i, then break
        #   - Else if we don't have time to go to the next intended pickup point, last ditch attempt is check if we have time
        #     to go to nearest pickup point and then home

        # Check if we're safe to go to pickup point and look for parcel
        print(f"Current location/direction at start of update: {self.robot.curr_node, self.robot.dir}")
        pickup_location = PICKUP_POINTS[self.i]
        print(f"Next pickup location: {pickup_location}")
        go_home_flag = True
        if self.check_safe_to_go(pickup_location):
            go_home_flag = False
            node_adj_to_pickup = GRAPH[pickup_location][0]
            self.robot.navigate(node_adj_to_pickup)
            self.robot.change_dir(self.robot.get_dir(node_adj_to_pickup, pickup_location), SHARP)

            dest_node = self.robot.pickup_parcel()

            print(f"The destination node is {dest_node}.")

            if dest_node in [DEPOT_RED_YELLOW, DEPOT_BLUE_GREEN]:
                if self.check_safe_to_go(dest_node):
                    self.robot.pickup_turn(dest_node)
                    self.robot.navigate(dest_node)
                    self.robot.depot_procedure(dest_node)
                else:
                    self.robot.pickup_turn(START_POINT)
                    go_home_flag = True

            else:
                next_pickup = PICKUP_POINTS[(self.i + 1) % 4]

                if self.check_safe_to_go(next_pickup):
                    print(next_pickup)
                    self.robot.pickup_turn(next_pickup)
                else:
                    self.robot.pickup_turn(START_POINT)
                    go_home_flag = True

                self.num_empty_parcel += 1
                if self.num_empty_parcel == 4:
                    go_home_flag = True

            self.i = (self.i + 1) % 4

        if go_home_flag:
            self.back_to_start()

    def check_safe_to_go(self, dest : tuple[int, int]) -> bool:
        time_to_dest, dest_dir = self.robot.time_for_path(self.robot.curr_node, dest, self.robot.dir)
        time_home, _ = self.robot.time_for_path(dest, START_POINT, dest_dir)

        time_now = ticks_diff(ticks_ms(), self.time_start) * 1e-03
        return time_now + time_to_dest + time_home < TOTAL_ALLOWED_TIME

    def back_to_start(self) -> None:
        self.robot.navigate(START_POINT)
