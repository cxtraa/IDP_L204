GRAPH = {
    (0,0) : [(103, 0), (-34, 0), (0, -29)],
    (0, -29) : [(0, 0)],
    (103, 0) : [(103, -31), (103, 88), (0,0)],
    (-34, 0) : [(-34, 32), (-104, 0), (0, 0)],
    (-104, 0) : [(-104, -31), (-104, 88), (-34, 0)],
    (-104, 88) : [(-104, 162), (-1, 88), (-104, 0)],
    (-34, 32) : [(-34, 0)],
    (-1, 88) : [(32, 88), (-1, 123), (-104, 88)],
    (32, 88) : [(-1, 88), (32, 66), (103, 88)],
    (103, 88) : [(103, 162), (103, 0), (32, 88)],
    (103, 162) : [(103, 88), (40, 162)],
    (40, 162) : [(103, 162), (-1, 162), (40, 139)],
    (40, 139) : [(40, 162)],
    (-1, 162) : [(40, 162), (-1, 123), (-104, 162)],
    (-104, 162) : [(-104, 88), (-1, 162)],
    (-27, 123) : [(-1, 123)],
    (-1, 123) : [(-27, 123), (-1, 162), (-1, 88)],
    (32, 66) : [(32, 88)],
    (-104, -31) : [(-104, 0)],
    (103, -31) : [(103, 0)], 
}

# Locations
PICKUP_POINTS = [(-34, 32), (32, 66), (-27, 123), (40, 139)]
DEPOT_RED_YELLOW = (-104, -31)
DEPOT_BLUE_GREEN = (103, -31)
START_POINT = (0,-29)

LEFT = 0
RIGHT = 1
FORWARDS = 0
BACKWARDS = 1

# Pin configuration
IR1_PIN = 11
IR2_PIN = 10
IR3_PIN = 9
IR4_PIN = 8
COLOR_SENSOR_PIN = 16
DIST_SENSOR_PIN = 20
LEFT_MOTOR_NUM = 3
RIGHT_MOTOR_NUM = 4
SERVO_NUM = 2
FLASH_LED_PIN = 20
COLOUR_SENSOR_SDA_PIN = 16
COLOUR_SENSOR_SCL_PIN = 17
TOF_SENSOR_SDA_PIN = 18
TOF_SENSOR_SCL_PIN = 19
START_BUTTON_PIN = 12

# Robot dimensions
ROBOT_EFFECTIVE_LENGTH = 115
ROBOT_EFFECTIVE_WIDTH = 175

SERVO_LIFTED_ANGLE = 45

PARCEL_DETECTION_THRESHOLD = 10  # Distance in mm to confirm parcel presence

# Line following
K_P = 20.0
K_D = 0.5
K_I = 0.001

# Time delays
TIME_FORWARD_AT_TURN = 1.35
TIME_FORWARD_AT_DEPOT = 0.4
EXTRA_REVERSE_TIME_AT_DEPOT = 0.1
TIGHT_SPACE_REVERSE_TIME = 0.2
DELTA_T = 0.001
TIME_SAFETY_FACTOR = 1.2
TOTAL_ALLOWED_TIME = 5 * 60

# Motor speeds
ROBOT_SPEED_TURN = 100
ROBOT_SPEED_LINE = 100
APPROX_LINE_SPEED = 1
ROBOT_SPEED_MISS_JUNCTION = 58
OUTSIDE_MOTOR_TURN_SPEED = 100
INSIDE_MOTOR_TURN_SPEED = (OUTSIDE_MOTOR_TURN_SPEED *
                           (2 * ROBOT_EFFECTIVE_LENGTH - ROBOT_EFFECTIVE_WIDTH) /
                           (2 * ROBOT_EFFECTIVE_LENGTH + ROBOT_EFFECTIVE_WIDTH))

SHARP = 0 # 90 degree sharp turn
SMOOTH = 1 # Smoother turn in arc

# Button constants
BUTTON_DEBOUNCE_TIME = 0.01

# Various sensor data
RED = 0
YELLOW = 1
GREEN = 2
BLUE = 3

COLOUR_READINGS = {
    (13, 9, 13, 36): RED, # Red
    (19, 24, 17, 64): YELLOW, # Yellow
    (7, 8, 12, 30): GREEN, # Green
    (9, 15, 30, 57): BLUE # Blue
}

SENSOR_POS = [-5.25, -0.95, 0.95, 5.25]
TOF_SENSOR_OFFSET = 55
