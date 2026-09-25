#!/usr/bin/python3
from vanh_msgs.msg import RobotInformation
from vanh_msgs.srv import ChangeMode

class ROBOT_JOINT_STATES:
    FRONT_LEFT_JOINT_1  = 0
    FRONT_LEFT_JOINT_2  = 1
    FRONT_LEFT_JOINT_3  = 2
    FRONT_RIGHT_JOINT_1 = 3
    FRONT_RIGHT_JOINT_2 = 4
    FRONT_RIGHT_JOINT_3 = 5
    BACK_LEFT_JOINT_1   = 6
    BACK_LEFT_JOINT_2   = 7
    BACK_LEFT_JOINT_3   = 8
    BACK_RIGHT_JOINT_1  = 9
    BACK_RIGHT_JOINT_2  = 10
    BACK_RIGHT_JOINT_3  = 11

class ROBOT_MODE:
    MODE_MANUAL_PS5  = 0
    MODE_AUTO    = 1
    MODE_PAUSE   = 2
    MODE_CHARGE_BATTERY = 3
    MODE_UNKNOWN = 100

class ROBOT_TEST_STAGE:
    NONE        = 0
    TRIGGER     = 1
    DONE        = 2


