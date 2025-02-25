from Motor import Motor
from time import sleep

def main():
    motor = Motor()
    motor.forward()
    time.sleep(1)
    motor.reverse()
    time.sleep(1)

if __name__ == "__main__":
    main()