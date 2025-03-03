from constants import *
from PathFinder import PathFinder
from Control import Control
from Motor import Motor
from Button import Button
from time import sleep
from  warnings import warn

class Robot:
    def __init__(self, graph: dict[tuple:list[tuple]], start_node=(0,0), start_dir=0, sensor_pos = []):
        """
        The possible robot directions are:
            - 0 North
            - 1 East
            - 2 South
            - 3 West
        """
        self.left_motor = Motor(LEFT_MOTOR_NUM)
        self.right_motor = Motor(RIGHT_MOTOR_NUM)
        self.start_button = Button(12)
        self.dir = start_dir
        self.curr_node = start_node
        self.graph = graph
        self.path_finder = PathFinder(graph=graph)
        
        self.control = Control(sensor_pos=sensor_pos)
    
    def navigate(self, dest : tuple[int, int]) -> None:
        """
        Move the robot to `dest` where multiple nodes might be in between.
        """
        shortest_path = self.path_finder.find_shortest_path(self.curr_node, dest)
        for i in range(1, len(shortest_path)):
            self.move(shortest_path[i])

    def forward(self) -> None:
        """
        Move the robot forward until it reaches the next node.
        """

        # Update internal robot state
        c_x, c_y = self.curr_node
        for neighbor in self.graph[self.curr_node]:
            n_x, n_y = neighbor
            cond1 = self.dir == 0 and n_y > c_y
            cond2 = self.dir == 1 and n_x > c_x
            cond3 = self.dir == 2 and n_y < c_y
            cond4 = self.dir == 3 and n_x < c_x
            if cond1 or cond2 or cond3 or cond4:
                self.curr_node = neighbor
                break

        # Move forward for half a second so we don't detect the last junction as a new one
        self.left_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
        self.right_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
        while self.control.at_junction():
            sleep(DELTA_T)

        # While we are not at a junction, run both the left and right motor, using PID control to line follow
        while not self.control.at_junction():
            self.left_motor.forward(ROBOT_SPEED_LINE + self.control.get_pid_error())
            self.right_motor.forward(ROBOT_SPEED_LINE - self.control.get_pid_error())
            sleep(DELTA_T)
        
        # The robot should be stationary after reaching the node
        self.left_motor.off()
        self.right_motor.off()
    
    def forward_turn_90(self, dir: int = 0) -> None:
        """Turn the robot 90 degrees in the direction indicated by dir.
        0 - left
        1 - right"""
        if dir == 0:
            outside_motor = self.right_motor
            inside_motor = self.left_motor
        elif dir == 1:
            outside_motor = self.left_motor
            inside_motor = self.right_motor
        else:
            raise(ValueError("Invalid direction: dir must be 0 (left) or 1 (right)"))
        
        outside_motor.forward(OUTSIDE_MOTOR_TURN_SPEED)
        inside_motor.forward(INSIDE_MOTOR_TURN_SPEED)
        while self.control.get_ir_readings()[1] or self.control.get_ir_readings()[2]:
            sleep(DELTA_T)
        while not (self.control.get_ir_readings()[1] and self.control.get_ir_readings()[2]):
            sleep(DELTA_T)
        outside_motor.off()
        inside_motor.off()
        
    def turn_left(self):
        """
        DEPRECATED: use forward_turn_90 instead.
        Turn the robot 90 deg anticlockwise.
        """
        warn("Deprecated: Use forward_turn_90 instead.", DeprecationWarning)
        self.dir = (-1 + self.dir) % 4

        # 1. Go forward a bit
        self.left_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
        self.right_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
        sleep(TIME_FORWARD_AT_TURN)
        self.left_motor.off()
        self.right_motor.off()

        # 2. Turn until neither of the middle sensors detect
        self.right_motor.forward(ROBOT_SPEED_TURN)
        self.left_motor.reverse(ROBOT_SPEED_TURN)
        while self.control.get_ir_readings()[1] or self.control.get_ir_readings()[2]:
            sleep(DELTA_T)
        
        # 3. Turn until both of the middle sensors detect
        while not (self.control.get_ir_readings()[1] and self.control.get_ir_readings()[2]):
            sleep(DELTA_T)
        
        self.right_motor.off()
        self.left_motor.off()

    def turn_right(self):
        """
        DEPRECATED: use forward_turn_90 instead.
        Make a 90 deg turn clockwise.
        This works by powering left wheel until a line is detected.
        """
        warn("Deprecated: Use forward_turn_90 instead.", DeprecationWarning)
        self.dir = (1 + self.dir) % 4

        # 1. Go forward a bit
        self.left_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
        self.right_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
        sleep(TIME_FORWARD_AT_TURN)
        self.left_motor.off()
        self.right_motor.off()

        # 2. Turn until neither of the middle sensors detect
        self.left_motor.forward(ROBOT_SPEED_TURN)
        self.right_motor.reverse(ROBOT_SPEED_TURN)
        while self.control.get_ir_readings()[1] or self.control.get_ir_readings()[2]:
            sleep(DELTA_T)
        
        # 3. Turn until both of the middle sensors detect
        while not (self.control.get_ir_readings()[1] and self.control.get_ir_readings()[2]):
            sleep(DELTA_T)
        
        self.left_motor.off()
        self.right_motor.off()

    def reverse_turn_90(self, dir: int = 0) -> None:
        """Turn the robot 90 degrees in the direction indicated by dir.
        0 - left
        1 - right"""
        if dir == 0:
            outside_motor = self.left_motor
            inside_motor = self.right_motor
        elif dir == 1:
            outside_motor = self.right_motor
            inside_motor = self.left_motor
        else:
            raise(ValueError("Invalid direction: dir must be 0 (left) or 1 (right)"))
        
        outside_motor.reverse(OUTSIDE_MOTOR_TURN_SPEED)
        inside_motor.reverse(INSIDE_MOTOR_TURN_SPEED)
        while self.control.get_ir_readings()[1] or self.control.get_ir_readings()[2]:
            sleep(DELTA_T)
        while not (self.control.get_ir_readings()[1] and self.control.get_ir_readings()[2]):
            sleep(DELTA_T)
        outside_motor.off()
        inside_motor.off()
    
    def turn_left_reverse(self) -> None:
        """
        DEPRECATED: use reverse_turn_90 instead.
        Perform a reverse turn that leaves the robot 90 deg anticlockwise from its original orientation.
        """
        warn("Deprecated: Use reverse_turn_90 instead.", DeprecationWarning)
        self.left_motor.reverse(ROBOT_SPEED_TURN)
        self.right_motor.forward(ROBOT_SPEED_TURN)
        while self.control.get_ir_readings()[1] or self.control.get_ir_readings()[2]:
            sleep(DELTA_T)
        while not (self.control.get_ir_readings()[1] and self.control.get_ir_readings()[2]):
            sleep(DELTA_T)
        self.left_motor.off()
        self.right_motor.off()
    
    def turn_right_reverse(self) -> None:
        """
        DEPRECATED: use reverse_turn_90 instead.
        Perform a reverse turn that leaves the robot 90 deg clockwise from its original orientation.
        """
        warn("Deprecated: Use reverse_turn_90 instead.", DeprecationWarning)
        self.right_motor.reverse(ROBOT_SPEED_TURN)
        self.left_motor.forward(ROBOT_SPEED_TURN)
        while self.control.get_ir_readings()[1] or self.control.get_ir_readings()[2]:
            sleep(DELTA_T)
        while not (self.control.get_ir_readings()[1] and self.control.get_ir_readings()[2]):
            sleep(DELTA_T)
        self.left_motor.off()
        self.right_motor.off()

    def turn_180_left(self) -> None:
        """
        Do a 180 deg turn anticlockwise.
        """
        self.right_motor.forward(ROBOT_SPEED_TURN)
        self.left_motor.reverse(ROBOT_SPEED_TURN)
        sleep(TIME_FOR_180_TURN)
    
    def turn_180_right(self) -> None:
        """
        Do a 180 deg turn clockwise.
        """
        self.left_motor.forward(ROBOT_SPEED_TURN)
        self.right_motor.reverse(ROBOT_SPEED_TURN)
        sleep(TIME_FOR_180_TURN)
    
    def change_dir(self, desired_dir : int):
        """
        Decide whether to call turn_left() or turn_right()
        """

        # We should only ever turn left or right (no 180 deg turns)
        if desired_dir == (self.dir + 1) % 4:
            self.forward_turn_90(0) # Turn right
        elif desired_dir == (self.dir - 1) % 4:
            self.forward_turn_90(1) # Turn left
        else:
            raise(ValueError("Invalid direction: must be 1 or -1 from current direction for change_dir()"))
    
    def move(self, dest : tuple[int, int]):
        """
        Move the robot from the current node to `dest`, where current node and dest are NEIGHBORS.

        1. Orient the robot in the direction it should move.
        2. Move forward.
        """
        x_1, y_1 = self.curr_node
        x_2, y_2 = dest

        if x_2 > x_1:
            self.change_dir(1)
        elif x_2 < x_1:
            self.change_dir(3)
        elif y_2 > y_1:
            self.change_dir(0)
        elif y_2 < y_1:
            self.change_dir(2)
        
        self.forward()

    def pickup_parcel(self) -> tuple[int, int] | None:
        """
        Robot procedure for picking up a parcel.
        0 - no parcel found
        1 - red/yellow
        2 - blue/green
        """
        
        dest_depot = DEPOT_RED_YELLOW # TODO detect colour via sensor and determine which depot to navigate to, return None if no package

        # We are at a pickup point, find the node before us (there is only 1) and move to it
        prev_node = GRAPH[self.curr_node][0]
        self.left_motor.reverse(ROBOT_SPEED_TURN)
        self.right_motor.reverse(ROBOT_SPEED_TURN)
        sleep(TIME_BACKWARDS_AFTER_PARCEL)

        # Find the next node on our path to the destination node to deliver the parcel
        path = self.path_finder.find_shortest_path(prev_node, dest_depot)
        next_node = path[1]

        # Perform the pickup turn based on the next node we need to reach
        new_dir = self.pickup_turn(next_node)
        self.dir = new_dir
        self.curr_node = prev_node

        return dest_depot
    
    def pickup_turn(self, node : tuple[int, int]) -> int:
        """
        Turn in the appropriate direction after collecting the parcel.
        """

        x1, y1 = self.curr_node
        x2, y2 = node

        # Decide whether to reverse left or reverse right
        left_cond1 = (self.dir == 0) and (x2 < x1)
        left_cond2 = (self.dir == 1) and (y2 > y1)
        left_cond3 = (self.dir == 2) and (x2 > x1)
        left_cond4 = (self.dir == 3) and (y2 < y1)
        reverse_left = left_cond1 or left_cond2 or left_cond3 or left_cond4

        right_cond1 = (self.dir == 0) and (x2 > x1)
        right_cond2 = (self.dir == 1) and (y2 < y1)
        right_cond3 = (self.dir == 2) and (x2 < x1)
        right_cond4 = (self.dir == 3) and (y2 > y1)
        reverse_right = right_cond1 or right_cond2 or right_cond3 or right_cond4

        if reverse_left:
            new_dir = (self.dir - 1) % 4
            self.reverse_turn_90(0) # Reverse turn left
        elif reverse_right:
            new_dir = (self.dir + 1) % 4
            self.reverse_turn_90(1) # Reverse turn right
        
        return new_dir

    def depot_procedure(self, next_node : tuple[int, int]) -> None:
        self.deposit_parcel() # Procedure for depositing the parcel

        if self.curr_node == (-104, 0): # bottom-left depot
            if next_node == (-104, 88):
                self.reverse_turn_90(1) # need to perform reverse turn
        elif self.curr_node == (103, 0): # bottom-right depot
            if next_node == (103, 88):
                self.reverse_turn_90(0)
        
        # From here, the robot can navigate normally.
    
    def deposit_parcel():
        """
        Robot procedure for depositing a parcel.
        """
        warn("Not implemented: deposit_parcel() is a placeholder function.", Warning)
        pass

    def __str__(self) -> str:
        directions = ["North", "East", "South", "West"]
        return f"Position: {self.curr_node} | Direction : {directions[self.dir]}"