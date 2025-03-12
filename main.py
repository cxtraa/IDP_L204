import sys
sys.path.append("/src")

from Robot import Robot
from constants import *
from StateMachine import StateMachine
from time import sleep

def main():
    state_machine = StateMachine()
    while not state_machine.robot.start_button.pressed():
        sleep(DELTA_T)
    
    state_machine.start_procedure() # move robot forward to junction
        
    while not state_machine.should_end:
        try:
            finished = state_machine.update()
            if finished:
                break
        except Exception as e:
            sys.print_exception(e)
            state_machine.back_to_start()
            break

    print("Program finished successfully.")

if __name__ == "__main__":
    main()