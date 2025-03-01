import sys
sys.path.append("/src")

from machine import Pin
from Robot import Robot
from constants import *
from StateMachine import StateMachine
from time import sleep

def main():
    state_machine = StateMachine()
    while not state_machine.should_end:
        state_machine.update()

if __name__ == "__main__":
    main()
