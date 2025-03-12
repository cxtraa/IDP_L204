from constants import *

from PathFinder import PathFinder
from Control import Control

from Motor import Motor
from Servo import Servo
from ColourSensor import ColourSensor
from TofSensor import TofSensor
from Button import Button
from FlashLed import FlashLed

from time import sleep, ticks_ms, ticks_diff


class Robot:
    def __init__(self, graph: dict[tuple:list[tuple]], sensor_pos, start_node=(0, -29), start_dir=0):
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
        self.servo.set_angle(45)

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

        self.total_line_distance = 0
        self.total_line_time = 0

        self.turn_time = 0
        self.control = Control(sensor_pos=sensor_pos)

    def navigate(self, dest: tuple[int, int]) -> None:
        """
        Move the robot to `dest` where multiple nodes might be in between.
        """
        shortest_path, _ = self.path_finder.find_shortest_path(self.curr_node, dest)
        for i in range(1, len(shortest_path)):
            self.move(shortest_path[i])

    def forward(self) -> None:
        """
        Move the robot forward until it reaches the next node.
        """
        # While we are not at a junction, run both the left and right motor, using PID control to line follow
        while not self.control.at_junction():
            self.left_motor.forward(ROBOT_SPEED_LINE + self.control.get_pid_error())
            self.right_motor.forward(ROBOT_SPEED_LINE - self.control.get_pid_error())
            sleep(DELTA_T)

        # The robot should be stationary after reaching the node
        self.left_motor.off()
        self.right_motor.off()

    def turn_90(self, direction: int, travel: int = FORWARDS, mode: int = SMOOTH) -> None:
        """Turn the robot 90 degrees in the direction indicated by dir.
        0 - left
        1 - right"""

        if not self.turn_time:
            turn_start_time = ticks_ms()

        if direction == LEFT:
            outside_motor = self.right_motor
            inside_motor = self.left_motor
        elif direction == RIGHT:
            outside_motor = self.left_motor
            inside_motor = self.right_motor
        else:
            raise (ValueError("Invalid direction: dir must be 0 (left) or 1 (right)"))

        if mode == SHARP:
            self.left_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
            self.right_motor.forward(ROBOT_SPEED_MISS_JUNCTION)
            sleep(TIME_FORWARD_AT_TURN)
            self.left_motor.off()
            self.right_motor.off()
        if mode == SHARP or travel == BACKWARDS:
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

        if not self.turn_time:
            turn_end_time = ticks_ms()
            self.turn_time = ticks_diff(turn_end_time, turn_start_time) / 1e03

    def turn_180(self, direction: int = LEFT) -> None:
        """
        Turn 180 degrees.
        """
        if direction == LEFT:
            outside_motor = self.right_motor
            inside_motor = self.left_motor
        elif direction == RIGHT:
            outside_motor = self.left_motor
            inside_motor = self.right_motor
            self.dir = (self.dir + 2) % 4
        else:
            raise ValueError("Invalid direction: direction must be 0 (Left) or 1 (Right)")

        inside_motor.reverse(ROBOT_SPEED_TURN)
        outside_motor.forward(ROBOT_SPEED_TURN)

        while self.control.get_ir_readings()[1] or self.control.get_ir_readings()[2]:
            sleep(DELTA_T)
        while not (self.control.get_ir_readings()[1] and self.control.get_ir_readings()[2]):
            sleep(DELTA_T)

    def change_dir(self, desired_dir: int, mode: int = SMOOTH) -> None:
        """
        Decide whether to call turn_left(), turn_right() or turn_180()
        """

        if desired_dir == (self.dir + 1) % 4:
            self.turn_90(RIGHT, FORWARDS, mode)  # Turn right
        elif desired_dir == (self.dir - 1) % 4:
            self.turn_90(LEFT, FORWARDS, mode)  # Turn left
        elif desired_dir == (self.dir + 2) % 4:
            self.turn_180()

        self.dir = desired_dir

    @staticmethod
    def get_dir(node_a, node_b):
        """
        Get the required orientation for the robot to go from node_A to node_B.
        """
        x_1, y_1 = node_a
        x_2, y_2 = node_b

        if x_2 > x_1:
            return 1
        elif x_2 < x_1:
            return 3
        elif y_2 > y_1:
            return 0
        else:
            return 2

    def move(self, dest: tuple[int, int]):
        """
        Move the robot from the current node to `dest`, where current node and dest are NEIGHBORS.

        1. Orient the robot in the direction it should move.
        2. Move forward.
        """
        x_1, y_1 = self.curr_node
        x_2, y_2 = dest

        desired_dir = self.get_dir(self.curr_node, dest)
        if dest in PICKUP_POINTS:
            mode = SHARP
        else:
            mode = SMOOTH
        self.change_dir(desired_dir, mode)
        # Start flashing led when leaving the start point
        if self.curr_node == START_POINT:
            self.flash_led.flash()

        line_start_time = ticks_ms()
        self.forward()
        line_end_time = ticks_ms()

        line_time = ticks_diff(line_end_time, line_start_time) / 1e03

        self.total_line_time += line_time
        self.total_line_distance += (abs(x_2 - x_1) + abs(y_2 - y_1)) / 1e02

        # Update current node
        self.curr_node = dest

        # Turn LED OFF if returning to the start
        if self.curr_node == START_POINT:
            self.flash_led.off()

    def time_for_path(self, node_a: tuple[int, int], node_b: tuple[int, int], start_dir: int) -> tuple[float, int]:
        """
        Calculate the time for the robot to reach the node `dest`.
        """
        path, distance = self.path_finder.find_shortest_path(node_a, node_b)
        if self.total_line_time == 0:
            line_speed = APPROX_LINE_SPEED
        else:
            line_speed = self.total_line_distance / self.total_line_time
        curr_dir = start_dir
        new_dir = curr_dir
        num_turns = 0

        # Find the number of turns made
        for i in range(1, len(path)):
            new_dir = self.get_dir(path[i - 1], path[i])
            if abs(curr_dir - new_dir) == 1:
                num_turns += 1
            elif abs(curr_dir - new_dir) == 2:
                num_turns += 2

        return TIME_SAFETY_FACTOR * ((num_turns * self.turn_time) + (distance * 1e-02) / line_speed), new_dir

    def get_depot_to_goto(self) -> tuple[int, int] | None:
        """
        can return:
        DEPOT_RED_YELLOW, DEPOT_BLUE_GREEN, None
        """
        parcel_colour = self.colour_sensor.read_colour()
        print(parcel_colour)

        if parcel_colour in [RED, YELLOW]:
            return DEPOT_RED_YELLOW
        elif parcel_colour in [BLUE, GREEN]:
            return DEPOT_BLUE_GREEN
        else:
            return None

    def pickup_parcel(self) -> tuple[int, int] | None:
        """
        Robot procedure for picking up a parcel. Returns the destination depot, or None if there is no parcel.
        0 - no parcel found
        1 - red/yellow
        2 - blue/green
        """
        self.servo.set_angle(0)
        sleep(0.5)
        start_time_forwards = ticks_ms()

        # Go forward until package detected or junction reached.
        while (self.tof_sensor.read_distance() > PARCEL_DETECTION_THRESHOLD) and not self.control.at_junction():
            self.left_motor.forward(ROBOT_SPEED_LINE + self.control.get_pid_error())
            self.right_motor.forward(ROBOT_SPEED_LINE - self.control.get_pid_error())
            sleep(DELTA_T)  # Allow time for sensor to update
        sleep(0.1)  # Short delay to account for noise in TOF reading
        self.left_motor.off()
        self.right_motor.off()

        end_time_forwards = ticks_ms()

        total_time_forwards = ticks_diff(end_time_forwards, start_time_forwards) / 1e03

        self.servo.set_angle(45)
        sleep(0.5)

        if self.tof_sensor.read_distance() <= PARCEL_DETECTION_THRESHOLD:
            dest_node = self.get_depot_to_goto()
        else:
            dest_node = None

        self.left_motor.reverse(ROBOT_SPEED_LINE)
        self.right_motor.reverse(ROBOT_SPEED_LINE)
        sleep(total_time_forwards)
        self.left_motor.off()
        self.right_motor.off()

        return dest_node

    def pickup_turn(self, node: tuple[int, int]):
        """
        Turn in the appropriate direction after collecting the parcel.
        """

        # Decide whether to reverse left or reverse right
        current_dir = self.dir
        target_dir = self.get_dir(self.curr_node, node)
        result = (target_dir - current_dir) % 4

        if result == 1:
            self.turn_90(RIGHT, BACKWARDS)
            self.dir = (self.dir + 1) % 4
        elif result == 3:
            self.turn_90(LEFT, BACKWARDS)
            self.dir = (self.dir - 1) % 4

    def depot_procedure(self, depot: tuple[int, int]) -> None:
        self.left_motor.forward(ROBOT_SPEED_LINE)
        self.right_motor.forward(ROBOT_SPEED_LINE)
        sleep(TIME_FORWARD_AT_DEPOT)

        self.deposit_parcel()

        self.left_motor.reverse(ROBOT_SPEED_LINE)
        self.right_motor.reverse(ROBOT_SPEED_LINE)
        while self.control.get_ir_readings()[0] or self.control.get_ir_readings()[3]:
            sleep(DELTA_T)
        sleep(EXTRA_REVERSE_TIME_AT_DEPOT)
        self.left_motor.off()
        self.right_motor.off()

        self.servo.set_angle(45)

        if depot == DEPOT_RED_YELLOW:
            self.turn_180(LEFT)
        elif depot == DEPOT_BLUE_GREEN:
            self.turn_180(RIGHT)
        self.dir = (self.dir + 2) % 4
        self.forward()
        self.curr_node = self.graph[depot][0]

    def deposit_parcel(self):
        """
        Robot procedure for depositing a parcel.
        """
        self.servo.set_angle(0)
        sleep(0.5)

    def __str__(self) -> str:
        directions = ["North", "East", "South", "West"]
        return f"Position: {self.curr_node} | Direction : {directions[self.dir]}"

