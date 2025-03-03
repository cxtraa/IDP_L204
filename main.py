import sys
sys.path.append("/src")

from machine import Pin
from Robot import Robot
from constants import *
from StateMachine import StateMachine
from time import sleep

def main():
    state_machine = StateMachine()
    while not state_machine.robot.start_button.pressed():
        sleep(DELTA_T)
        
    while not state_machine.should_end:
        try:
            state_machine.update()
        except Exception as e:
            sys.print_exception(e)
            break
    state_machine.stop()
    print("Finished")

if __name__ == "__main__":
    main()