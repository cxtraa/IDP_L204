from constants import *
from PathFinder import PathFinder
from Control import Control
from Motor import Motor
from Button import Button
from time import sleep

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
        
    def turn_left(self):
        """
        Turn the robot 90 deg anticlockwise.
        """
        self.dir = (-1 + self.dir) % 4

        # 1. Go forward a bit
        self.left_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
        self.right_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
        sleep(TIME_FORWARD_AT_TURN)
        self.left_motor.off()
        self.right_motor.off()

        # 2. Turn until neither of the middle sensors detect
        self.right_motor.forward(ROBOT_SPEED_TURN)
        self.left_motor.reverse(40)
        while self.control.get_ir_readings()[1] or self.control.get_ir_readings()[2]:
            sleep(DELTA_T)
        
        # 3. Turn until both of the middle sensors detect
        while not (self.control.get_ir_readings()[1] and self.control.get_ir_readings()[2]):
            sleep(DELTA_T)
        
        self.right_motor.off()
        self.left_motor.off()

    def turn_right(self):
        """
        Make a 90 deg turn clockwise.
        This works by powering left wheel until a line is detected.
        """
        self.dir = (1 + self.dir) % 4

        # 1. Go forward a bit
        self.left_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
        self.right_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
        sleep(TIME_FORWARD_AT_TURN)
        self.left_motor.off()
        self.right_motor.off()

        # 2. Turn until neither of the middle sensors detect
        self.left_motor.forward(ROBOT_SPEED_TURN)
        self.right_motor.reverse(40)
        while self.control.get_ir_readings()[1] or self.control.get_ir_readings()[2]:
            sleep(DELTA_T)
        
        # 3. Turn until both of the middle sensors detect
        while not (self.control.get_ir_readings()[1] and self.control.get_ir_readings()[2]):
            sleep(DELTA_T)
        
        self.left_motor.off()
        self.right_motor.off()
    
    def reverse_left(self) -> None:
        """
        Perform a reverse turn that leaves the robot 90 deg anticlockwise from its original orientation.
        """
        self.left_motor.reverse(ROBOT_SPEED_TURN)
        self.right_motor.forward(ROBOT_SPEED_TURN)
        while self.control.get_ir_readings()[1] or self.control.get_ir_readings()[2]:
            sleep(DELTA_T)
        while not (self.control.get_ir_readings()[1] and self.control.get_ir_readings()[2]):
            sleep(DELTA_T)
        self.left_motor.off()
        self.right_motor.off()
    
    def reverse_right(self) -> None:
        """
        Perform a reverse turn that leaves the robot 90 deg clockwise from its original orientation.
        """
        self.right_motor.reverse(ROBOT_SPEED_TURN)
        self.left_motor.forward(ROBOT_SPEED_TURN)
        while self.control.get_ir_readings()[1] or self.control.get_ir_readings()[2]:
            sleep(DELTA_T)
        while not (self.control.get_ir_readings()[1] and self.control.get_ir_readings()[2]):
            sleep(DELTA_T)
        self.left_motor.off()
        self.right_motor.off()
    
    def change_dir(self, desired_dir : int):
        """
        Decide whether to call turn_left() or turn_right()
        """

        # We should only ever turn left or right (no 180 deg turns)
        if desired_dir == (self.dir + 1) % 4:
            self.turn_right()
        elif desired_dir == (self.dir - 1) % 4:
            self.turn_left()
    
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

    def pickup(self, curr_node : tuple[int, int], dest_node : tuple[int, int]) -> None:
        
        # We are at a pickup point, find the node before us (there is only 1) and move to it
        prev_node = GRAPH[curr_node][0]
        self.left_motor.reverse(ROBOT_SPEED_TURN)
        self.right_motor.reverse(ROBOT_SPEED_TURN)
        sleep(TIME_BACKWARDS_AFTER_PARCEL)

        # Find the next node on our path to the destination node to deliver the parcel
        path = self.path_finder.find_shortest_path(prev_node, dest_node)
        next_node = path[1]

        # Perform the pickup turn based on the next node we need to reach
        self.pickup_turn(next_node)
    
    def pickup_turn(self, node : tuple[int, int]) -> None:
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
            self.reverse_left()
        elif reverse_right:
            self.reverse_right()

    def __str__(self) -> str:
        directions = ["North", "East", "South", "West"]
        return f"Position: {self.curr_node} | Direction : {directions[self.dir]}"