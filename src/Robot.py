from constants import *
from PathFinder import PathFinder
from Control import Control
from Motor import Motor
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
        self.left_motor.forward(ROBOT_SPEED_SLOW)
        self.right_motor.forward(ROBOT_SPEED_SLOW)
        sleep(TIME_FORWARD_AT_TURN)

        # While we are not at a junction, run both the left and right motor, using PID control to line follow
        while not self.control.at_junction():
            self.left_motor.forward(ROBOT_SPEED_FAST + self.control.get_pid_error())
            self.right_motor.forward(ROBOT_SPEED_FAST - self.control.get_pid_error())
            sleep(DELTA_T)
        
        # The robot should be stationary after reaching the node
        self.left_motor.off()
        self.right_motor.off()


    def turn_right(self):
        """
        Make a 90 deg turn clockwise.
        This works by powering left wheel until a line is detected.
        """
        self.dir = (1 + self.dir) % 4

        self.left_motor.forward(ROBOT_SPEED_SLOW)
        self.right_motor.forward(ROBOT_SPEED_SLOW)
        sleep(TIME_FORWARD_AT_TURN)
        self.left_motor.off()
        self.right_motor.off()

        self.left_motor.forward(ROBOT_SPEED_SLOW)
        sleep(0.5) # In case there is a path forwards and right, don't detect the forwards path as finishing the turn
        while not self.control.__tracker_sensors[1].read():
            sleep(0.01)
        
        self.left_motor.off()
        
    def turn_left(self):
        """
        Turn the robot 90 deg anticlockwise.
        """
        self.dir = (-1 + self.dir) % 4

        self.left_motor.forward(ROBOT_SPEED_SLOW)
        self.right_motor.forward(ROBOT_SPEED_SLOW)
        sleep(TIME_FORWARD_AT_TURN)
        self.left_motor.off()
        self.right_motor.off()

        self.right_motor.forward(ROBOT_SPEED_SLOW)
        sleep(0.5) # In case there is a path forwards and right, don't detect the forwards path as finishing the turn
        while not self.control.__tracker_sensors[2].read():
            sleep(0.01)
        self.right_motor.off()
    
    def change_dir(self, desired_dir : int):
        """
        Find the optimal turns to change the direction.
        E.g. if going from North to East, we should call turn_right() once.
        """

        if desired_dir == (self.dir + 1) % 4:
            self.turn_right()
        elif desired_dir == (self.dir - 1) % 4:
            self.turn_left()
        elif desired_dir == (self.dir + 2) % 4:
            self.turn_right()
            self.turn_right()
    
    def move(self, dest : tuple[int, int]):
        """
        Move the robot from the current node to `dest`, where current node and dest are NEIGHBORS.
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

    def __str__(self) -> str:
        directions = ["North", "East", "South", "West"]
        return f"Position: {self.curr_node} | Direction : {directions[self.dir]}"