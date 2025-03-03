from constants import *
from PathFinder import PathFinder
from Control import Control
from Motor import Motor
from Button import Button
from FlashLed import FlashLed
from ColourSensor import ColourSensor
from TofSensor import TofSensor
from time import sleep, sleep_ms, ticks_ms, ticks_diff
from  warnings import warn

# Forward declaration of state machine
class StateMachine:
    pass

class Robot:
    def __init__(self, graph: dict[tuple:list[tuple]], start_node=(0,0), start_dir=0, sensor_pos = [], state_machine : StateMachine = None):
        """
        The possible robot directions are:
            - 0 North
            - 1 East
            - 2 South
            - 3 West
        """
        self.left_motor = Motor(LEFT_MOTOR_NUM)
        self.right_motor = Motor(RIGHT_MOTOR_NUM)
        self.flash_led = FlashLed(FLASH_LED_PIN)
        try:
            self.colour_sensor = ColourSensor(COLOUR_SENSOR_SDA_PIN, COLOUR_SENSOR_SCL_PIN)
        except OSError:
            self.colour_sensor = None
        try:
            self.tof_sensor = TofSensor(TOF_SENSOR_SDA_PIN, TOF_SENSOR_SCL_PIN)
        except OSError:
            self.tof_sensor = None
        self.start_button = Button(START_BUTTON_PIN)
        self.dir = start_dir
        self.curr_node = start_node
        self.graph = graph
        self.path_finder = PathFinder(graph=graph)

        self.__last_time_slow_pickup = 0
        self.__last_time_fast_pickup = 0
        
        self.control = Control(sensor_pos=sensor_pos)
    
    def navigate(self, dest : tuple[int, int]) -> None:
        """
        Move the robot to `dest` where multiple nodes might be in between.
        """
        shortest_path, _ = self.path_finder.find_shortest_path(self.curr_node, dest)
        for i in range(1, len(shortest_path)):
            self.move(shortest_path[i])
            print(self.curr_node)

    def forward(self) -> None:
        """
        Move the robot forward until it reaches the next node.
        """

        # Update internal robot state
        c_x, c_y = self.curr_node

        from_start_flag = (self.curr_node == START_POINT)

        for neighbor in self.graph[self.curr_node]:
            n_x, n_y = neighbor
            cond1 = self.dir == 0 and n_y > c_y
            cond2 = self.dir == 1 and n_x > c_x
            cond3 = self.dir == 2 and n_y < c_y
            cond4 = self.dir == 3 and n_x < c_x
            if cond1 or cond2 or cond3 or cond4:
                self.curr_node = neighbor
                break
        
        to_start_flag = (self.curr_node == START_POINT)

        if self.curr_node in PICKUP_POINTS:
            start_slow_pickup = ticks_ms()

        # Move forward until we don't detect the last junction
        self.left_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
        self.right_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
        while self.control.at_junction():
            sleep(DELTA_T)

        if self.curr_node in PICKUP_POINTS:
            end_slow_pickup = ticks_ms()
            self.__last_time_slow_pickup = ticks_diff(end_slow_pickup, start_slow_pickup)
            start_fast_pickup = ticks_ms()

        # While we are not at a junction, run both the left and right motor, using PID control to line follow
        while not self.control.at_junction():
            self.left_motor.forward(ROBOT_SPEED_LINE + self.control.get_pid_error())
            self.right_motor.forward(ROBOT_SPEED_LINE - self.control.get_pid_error())
            sleep(DELTA_T)
        
        # The robot should be stationary after reaching the node
        self.left_motor.off()
        self.right_motor.off()

        if self.curr_node in PICKUP_POINTS:
            end_fast_pickup = ticks_ms()
            self.__last_time_fast_pickup = ticks_diff(end_fast_pickup, start_fast_pickup)

        if from_start_flag: # Turn on the LED if we have just left the starting node
            self.flash_led.flash()
        elif to_start_flag: # Else, turn off the LED if we are going to the starting node
            self.flash_led.off()
    
    def forward_turn_90(self, dir: int, mode : int = SMOOTH) -> None:
        """Turn the robot 90 degrees in the direction indicated by dir.
        0 - left
        1 - right"""
        if dir == LEFT:
            outside_motor = self.right_motor
            inside_motor = self.left_motor
            self.dir = (self.dir - 1) % 4
        elif dir == RIGHT:
            outside_motor = self.left_motor
            inside_motor = self.right_motor
            self.dir = (self.dir + 1) % 4
        else:
            raise(ValueError("Invalid direction: dir must be 0 (left) or 1 (right)"))
        
        if mode == SHARP:
            self.left_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
            self.right_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
            sleep(TIME_FORWARD_AT_TURN)
            self.left_motor.off()
            self.right_motor.off()
            outside_motor.forward(ROBOT_SPEED_TURN)
            inside_motor.reverse(ROBOT_SPEED_TURN)
        elif mode == SMOOTH:
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

    def reverse_turn_90(self, dir: int) -> None:
        """Turn the robot 90 degrees in the direction indicated by dir."""
        if dir == LEFT:
            outside_motor = self.right_motor
            inside_motor = self.left_motor
            self.dir = (self.dir - 1) % 4
        elif dir == RIGHT:
            outside_motor = self.left_motor
            inside_motor = self.right_motor
            self.dir = (self.dir + 1) % 4
        else:
            raise(ValueError("Invalid direction: dir must be 0 (left) or 1 (right)"))
        
        outside_motor.forward(ROBOT_SPEED_TURN)
        inside_motor.reverse(ROBOT_SPEED_TURN)

        while self.control.get_ir_readings()[1] or self.control.get_ir_readings()[2]:
            sleep(DELTA_T)
        while not (self.control.get_ir_readings()[1] and self.control.get_ir_readings()[2]):
            sleep(DELTA_T)
        outside_motor.off()
        inside_motor.off()

    def turn_180(self, dir : int = LEFT) -> None:
        """
        Turn 180 degrees.
        """
        if dir == LEFT:
            outside_motor = self.right_motor
            inside_motor = self.left_motor
            self.dir = (self.dir - 2) % 4
        elif dir == RIGHT:
            outside_motor = self.left_motor
            inside_motor = self.right_motor
            self.dir = (self.dir + 2) % 4

        inside_motor.reverse(ROBOT_SPEED_TURN)
        outside_motor.forward(ROBOT_SPEED_TURN)

        while self.control.get_ir_readings()[1] or self.control.get_ir_readings()[2]:
            sleep(DELTA_T)
        while not (self.control.get_ir_readings()[1] and self.control.get_ir_readings()[2]):
            sleep(DELTA_T)
    
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

    def change_dir(self, desired_dir : int, mode : int = SMOOTH) -> None:
        """
        Decide whether to call turn_left(), turn_right() or turn_180()
        """

        if desired_dir == (self.dir + 1) % 4:
            self.forward_turn_90(RIGHT, mode) # Turn right
        elif desired_dir == (self.dir - 1) % 4:
            self.forward_turn_90(LEFT, mode) # Turn left
        elif desired_dir == (self.dir + 2) % 4:
            self.turn_180()

    def get_dir(self, node_A, node_B):
        """
        Get the required orientation for the robot to go from node_A to node_B.
        """
        x_1, y_1 = node_A
        x_2, y_2 = node_B

        if x_2 > x_1:
            return 1
        elif x_2 < x_1:
            return 3
        elif y_2 > y_1:
            return 0
        else:
            return 2

    def move(self, dest : tuple[int, int]):
        """
        Move the robot from the current node to `dest`, where current node and dest are NEIGHBORS.

        1. Orient the robot in the direction it should move.
        2. Move forward.
        """
        x_1, y_1 = self.curr_node
        x_2, y_2 = dest

        turn_mode = SHARP if (dest in PICKUP_POINTS) else SMOOTH

        desired_dir = self.get_dir(self.curr_node, dest)
        self.change_dir(desired_dir, turn_mode)
        
        self.forward()

        # Update timings
        curr_time = ticks_ms()
        self.state_machine.t += curr_time - self.prev_time
        self.prev_time = curr_time

        # Compute the amount of time to return to start
    
    def time_to_start(self) -> None:
        """
        Calculate the time for the robot to return to the start.
        """
        _, distance = self.path_finder.find_shortest_path(self.curr_node, START_POINT)
        return TIME_SAFETY_FACTOR * (distance * (10e-02)) / TRUE_SPEED_LINE

    def get_depot_to_goto(self) -> tuple[int, int] | None:
        """
        TODO: color sensing logic should go here.
        can return:
        DEPOT_RED_YELLOW, DEPOT_BLUE_GREEN, None
        """
        return DEPOT_RED_YELLOW

    def pickup_parcel(self, next_pickup_location : tuple[int, int]) -> tuple[int, int] | None:
        """
        Robot procedure for picking up a parcel. Returns the destination depot, or None if there is no parcel.
        0 - no parcel found
        1 - red/yellow
        2 - blue/green
        """
        
        dest_node = self.get_depot_to_goto()
        if not dest_node:
            dest_node = next_pickup_location

        # We are at a pickup point, find the node before us (there is only 1) and move to it
        prev_node = GRAPH[self.curr_node][0]
        self.left_motor.reverse(ROBOT_SPEED_TURN)
        self.right_motor.reverse(ROBOT_SPEED_TURN)
        sleep_ms(self.__last_time_fast_pickup)
        self.left_motor.reverse(ROBOT_SPEED_MISS_JUNCTION)
        self.right_motor.reverse(ROBOT_SPEED_MISS_JUNCTION)
        sleep_ms(self.__last_time_slow_pickup)
        self.curr_node = prev_node

        # Find the next node on our path to the destination node to deliver the parcel
        path = self.path_finder.find_shortest_path(self.curr_node, dest_node)
        next_node = path[1]

        # Perform the pickup turn based on the next node we need to reach
        self.pickup_turn(next_node)

        print(f"Currently at {self.curr_node}")
        print(f"Direction: {self.dir}")

        return dest_node
    
    def pickup_turn(self, node : tuple[int, int]):
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
            self.reverse_turn_90(LEFT)
        elif reverse_right:
            self.reverse_turn_90(RIGHT)

    def depot_procedure(self, depot : int) -> None:

        self.deposit_parcel()
        self.left_motor.forward(ROBOT_SPEED_LINE)
        self.right_motor.forward(ROBOT_SPEED_LINE)
        sleep(TIME_FORWARD_AT_DEPOT)
        
        self.left_motor.reverse(ROBOT_SPEED_TURN)
        self.right_motor.reverse(ROBOT_SPEED_TURN)
        while self.control.get_ir_readings()[1] or self.control.get_ir_readings()[2]:
            sleep(DELTA_T)
        self.left_motor.off()
        self.right_motor.off()

        if depot == DEPOT_RED_YELLOW:
            self.turn_180(LEFT)
        elif depot == DEPOT_BLUE_GREEN:
            self.turn_180(RIGHT)
        self.forward()
    
    def deposit_parcel(self):
        """
        Robot procedure for depositing a parcel.
        """
        # warn("Not implemented: deposit_parcel() is a placeholder function.", Warning)
        pass

    def __str__(self) -> str:
        directions = ["North", "East", "South", "West"]
        return f"Position: {self.curr_node} | Direction : {directions[self.dir]}"