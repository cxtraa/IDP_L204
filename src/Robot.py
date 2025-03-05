from constants import *

from PathFinder import PathFinder
from Control import Control

from Motor import Motor
from Servo import Servo
from ColourSensor import ColourSensor
from TofSensor import TofSensor
from Button import Button
from FlashLed import FlashLed

from time import sleep, sleep_ms, ticks_ms, ticks_diff
import numpy as np


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
        self.servo = Servo(SERVO_NUM)
        self.servo.set_angle(0)

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


    def forward(self, to_pickup: bool = False) -> None:
        """
        Move the robot forward until it reaches the next node.
        """

        if to_pickup:
            start_slow_pickup = ticks_ms()

        # Move forward until we don't detect the last junction
        self.left_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
        self.right_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
        while self.control.at_junction():
            sleep(DELTA_T)

        if to_pickup:
            end_slow_pickup = start_fast_pickup = ticks_ms()
            self.__last_time_slow_pickup = ticks_diff(end_slow_pickup, start_slow_pickup)

        # While we are not at a junction, run both the left and right motor, using PID control to line follow
        while not self.control.at_junction():
            self.left_motor.forward(ROBOT_SPEED_LINE + self.control.get_pid_error())
            self.right_motor.forward(ROBOT_SPEED_LINE - self.control.get_pid_error())
            sleep(DELTA_T)
        
        # The robot should be stationary after reaching the node
        self.left_motor.off()
        self.right_motor.off()

        if to_pickup:
            end_fast_pickup = ticks_ms()
            self.__last_time_fast_pickup = ticks_diff(end_fast_pickup, start_fast_pickup)
    

    def forward_turn_90(self, dir: int, mode : int = SMOOTH) -> None:
        """Turn the robot 90 degrees in the direction indicated by dir.
        0 - left
        1 - right"""
        if dir == LEFT:
            outside_motor = self.right_motor
            inside_motor = self.left_motor
        elif dir == RIGHT:
            outside_motor = self.left_motor
            inside_motor = self.right_motor
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


    def reverse_turn_90(self, dir: int) -> None:
        """Turn the robot 90 degrees in the direction indicated by dir."""
        if dir == LEFT:
            outside_motor = self.right_motor
            inside_motor = self.left_motor
        elif dir == RIGHT:
            outside_motor = self.left_motor
            inside_motor = self.right_motor
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

        self.dir = desired_dir


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
        # Start flasing led when leaving the start point
        if self.curr_node == START_POINT:
            self.flash_led.flash()
        self.forward(to_pickup=(dest in PICKUP_POINTS))

        # Update current node
        self.curr_node = dest

        # Turn LED OFF if returning to the start
        if self.curr_node == START_POINT:
            self.flash_led.off()

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
        parcel_colour = self.colour_sensor.read_colour()

        if parcel_colour in [RED, YELLOW]:
            return DEPOT_RED_YELLOW
        elif parcel_colour in [BLUE, GREEN]:
            return DEPOT_BLUE_GREEN
        else:
            return None


    def pickup_parcel(self, next_pickup_location : tuple[int, int]) -> tuple[int, int]:
        """
        Robot procedure for picking up a parcel. Returns the destination depot, or None if there is no parcel.
        0 - no parcel found
        1 - red/yellow
        2 - blue/green
        """

        # TODO: Use TOF sensor to detect parcel and make sure we are close enough to pick it up
        
        dest_node = self.get_depot_to_goto()
        if dest_node is None: # No parcel found
            dest_node = next_pickup_location
        else:
            self.servo.set_angle(30)
            sleep(0.5)

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

        return dest_node
    

    def pickup_turn(self, node : tuple[int, int]):
        """
        Turn in the appropriate direction after collecting the parcel.
        """

        x1, y1 = self.curr_node
        x2, y2 = node

        # Decide whether to reverse left or reverse right
        direction_matrix = np.array([
            [0, 1, 0],  # North
            [1, 0, 0],   # East
            [0, -1, 0],   # South
            [-1, 0, 0]   # West
        ])

        current_dir = direction_matrix[self.dir]
        target_dir = np.array([x2 - x1, y2 - y1, 0])

        result = np.cross(current_dir, target_dir)[2]

        if result < 0:
            self.reverse_turn_90(RIGHT)
            self.dir = (self.dir + 1) % 4
        elif result > 0:
            self.reverse_turn_90(LEFT)
            self.dir = (self.dir - 1) % 4


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
        self.servo.set_angle(0)
        sleep(0.5)


    def __str__(self) -> str:
        directions = ["North", "East", "South", "West"]
        return f"Position: {self.curr_node} | Direction : {directions[self.dir]}"
