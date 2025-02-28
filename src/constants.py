"""
Contains:
- Graph of map
- Pin names to numbers
- Movement parameters such as K_P, K_I, K_D
"""

# Graph of map robot will go on
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

# Pin configuration
IR1_PIN = 8
IR2_PIN = 9
IR3_PIN = 10
IR4_PIN = 11
COLOR_SENSOR_PIN = 16
DIST_SENSOR_PIN = 20
LEFT_MOTOR_NUM = 3
RIGHT_MOTOR_NUM = 4

# Movement parameters
K_P = 15.0
K_D = 1.0
K_I = 0.01
TIME_FORWARD_AT_TURN = 1.5
TIME_BACKWARDS_AFTER_PARCEL = 2.0
DELTA_T = 0.001
ROBOT_SPEED_LINE = 85
ROBOT_SPEED_TURN = 70
ROBOT_SPEED_MISS_JUNCTION = 50

# Button
BUTTON_DEBOUNCE_TIME = 0.01

# Significant points
pickup_1 = (-34,32)
pickup_2 = (32,66)
pickup_3 = (-27,123)
pickup_4 = (40,139)

start_point = (0,-29)

depot_1 = (103,-31)
depot_2 = (-104,-31)