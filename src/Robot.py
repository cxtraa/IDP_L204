from constants import GRAPH
from PathFinder import PathFinder
from Control import Control
from Motor import Motor
from time import sleep

class Robot:
    def __init__(self, graph, start_node=(0,0), start_dir=0):
        """
        The possible robot directions are:
            - 0 North
            - 1 East
            - 2 South
            - 3 West
        """
        self.dir = start_dir
        self.curr_node = start_node
        self.graph = graph
        self.path_finder = PathFinder(graph=graph)
        
        self.line_follower = Control()
        self.left_motor = Motor(3)
        self.right_motor = Motor(4)
    
    def navigate(self, dest):
        """
        Move the robot to `dest` where multiple nodes might be in between.
        """
        shortest_path = self.path_finder.find_shortest_path(self.curr_node, dest)
        print(shortest_path)
        for i in range(1, len(shortest_path)):
            self.move(shortest_path[i])

    def forward(self) -> None:
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

        # While we are not at a junction, run both the left and right motor
        while not self.line_follower.check_junction():
            self.left_motor.forward()
            self.right_motor.forward()
            sleep(0.1)
        
        self.left_motor.off()
        self.right_motor.off()


    def turn_right(self):
        self.dir = (1 + self.dir) % 4

        # TODO: Implement actual turning right
        self.left_motor.forward()
        sleep(0.1)
        
        self.left_motor.off()
        
    def turn_left(self):
        self.dir = (-1 + self.dir) % 4

        # TODO: Implement actual turning left
    
    def change_dir(self, desired_dir):
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
    
    def move(self, dest):
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

def main():
    robot = Robot(graph=GRAPH, start_node=(103,0), start_dir=1)
    robot.navigate((-104, 162))

if __name__ == "__main__":
    main()