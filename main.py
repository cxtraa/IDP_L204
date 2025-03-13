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
            print("There was an exception, going back to start.")
            state_machine.robot.back_to_start()
            break

    state_machine.end_procedure()

    print("Program finished successfully.")

if __name__ == "__main__":
    main()