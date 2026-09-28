#!/usr/bin/python3
from vanh_ros_control.robot_define import ROBOT_JOINT_STATES

from math import pi, sqrt
from time import sleep, time
from typing import Tuple

class ROBOT_LIMIT:
    FL_HIP = (-0.349066, 0.349066)
    FL_THIGH = (-1.22173, 0.0)
    FL_CALF = (-0.872665, 0.872665)

    FR_HIP = (-0.349066, 0.349066)
    FR_THIGH = (0.0, 1.22173)
    FR_CALF = (-0.872665, 0.872665)

    BL_HIP = (-0.349066, 0.349066)
    BL_THIGH = (-1.22173, 0.0)
    BL_CALF = (-0.872665, 0.872665)

    BR_HIP = (-0.349066, 0.349066)
    BR_THIGH = (0.0, 1.22173)
    BR_CALF = (-0.872665, 0.872665)

    JOINTS_ORDER = [
        FL_HIP, FL_THIGH, FL_CALF,
        FR_HIP, FR_THIGH, FR_CALF,
        BL_HIP, BL_THIGH, BL_CALF,
        BR_HIP, BR_THIGH, BR_CALF,
    ]


class ROBOT_SPEED:
    FL_HIP = 6.5
    FL_THIGH = 6.5
    FL_CALF = 6.5

    FR_HIP = 6.5
    FR_THIGH = 6.5
    FR_CALF = 6.5

    BL_HIP = 6.5
    BL_THIGH = 6.5
    BL_CALF = 6.5

    BR_HIP = 6.5
    BR_THIGH = 6.5
    BR_CALF = 6.5

    JOINTS_ORDER = [
        FL_HIP, FL_THIGH, FL_CALF,
        FR_HIP, FR_THIGH, FR_CALF,
        BL_HIP, BL_THIGH, BL_CALF,
        BR_HIP, BR_THIGH, BR_CALF,
    ]

class Sim_Interface:
    def __init__(self) -> None:
        self.mode = 255
        self.joint_action = [0.0] * ROBOT_JOINT_STATES.NUM_JOINTS
        self.joint_state = [0.0] * ROBOT_JOINT_STATES.NUM_JOINTS

        self.__manual = [0.0] * ROBOT_JOINT_STATES.NUM_JOINTS
        self.__auto = [0.0] * ROBOT_JOINT_STATES.NUM_JOINTS
        self.__fake = [0.0] * ROBOT_JOINT_STATES.NUM_JOINTS
        self.__target = [0.0] * ROBOT_JOINT_STATES.NUM_JOINTS

    # METHOD
    def connected(self) -> bool:
        """
        Check connect to stm32
        """
        return True

    def setMode(self, mode: ChangeMode) -> Tuple["bool|str"]:
        """
        Set Controller mode
        """
        self.mode = mode
        return True, ""