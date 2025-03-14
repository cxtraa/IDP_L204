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
SERVO_NUM = 1
FLASH_LED_PIN = 20
COLOUR_SENSOR_SDA_PIN = 16
COLOUR_SENSOR_SCL_PIN = 17
TOF_SENSOR_SDA_PIN = 18
TOF_SENSOR_SCL_PIN = 19
START_BUTTON_PIN = 12

# Robot dimensions
ROBOT_LENGTH = 165
ROBOT_WIDTH = 190

PARCEL_DETECTION_THRESHOLD = 50  # Distance in mm to confirm parcel presence

# Line following
K_P = 10.0
K_D = 0.5
K_I = 0.001

# Time delays
TIME_FORWARD_AT_TURN = 1.0
TIME_FORWARD_AT_DEPOT = 0.8
TIME_BACKWARDS_AFTER_PARCEL = 1.5
DELTA_T = 0.001
TIME_SAFETY_FACTOR = 1.2
TOTAL_ALLOWED_TIME = 5 * 60

# Motor speeds
ROBOT_SPEED_TURN = 100
ROBOT_SPEED_LINE = 85
ROBOT_SPEED_MISS_JUNCTION = 58
ROBOT_SPEED_APPROACHING_PARCEL = 30
OUTSIDE_MOTOR_TURN_SPEED = 100
INSIDE_MOTOR_TURN_SPEED = OUTSIDE_MOTOR_TURN_SPEED * (2 * ROBOT_LENGTH - ROBOT_WIDTH) / (2 * ROBOT_LENGTH + ROBOT_WIDTH)

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
    (2783.53, -1.05657): RED, # Red
    (8890.354, 0.5218): YELLOW, # Yellow
    (1672.447, -0.73191): GREEN, # Green
    (4518.424, 0.11455): BLUE # Blue
}

SENSOR_POS = [-5.25, -0.75, 0.75, 5.25]
TOF_SENSOR_OFFSET = 55
