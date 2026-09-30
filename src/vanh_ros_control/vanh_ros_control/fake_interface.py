#!/usr/bin/python3
from vanh_ros_control.utils import runInThread
from vanh_ros_control.robot_define import ROBOT_JOINT_STATES
from vanh_msgs.srv import ChangeMode

from math import isfinite, pi, sqrt
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
        self.version = 1 
        self.error = []                   
        self.device = [False] * 2

        self.joint_state = [0.0] * ROBOT_JOINT_STATES.NUM_JOINTS

        self.__target = [0.0] * ROBOT_JOINT_STATES.NUM_JOINTS
        self.__updateStatus()

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

    def controlDevice(self, indexs: list, enables: list) -> bool:
        """
        Control devices
        """
        for i in range(indexs.__len__()):
            self.device[indexs[i]] = enables[i]
        return True
    
    def controlManualJoint(self, positions: list) -> bool:
        """
        Control joints manually (joints in radian)
        """
        if len(positions) != ROBOT_JOINT_STATES.NUM_JOINTS:
            print(f"Invalid joint positions length: {len(positions)}, expected: {ROBOT_JOINT_STATES.NUM_JOINTS}")
            return False
        
        for i in range(ROBOT_JOINT_STATES.NUM_JOINTS):
            angle = positions[i]
            low, high = ROBOT_LIMIT.JOINTS_ORDER[i]
            if not isfinite(angle) or angle < low or angle > high:
                print(f"Invalid joint angle: {angle} for joint {i}, limits: {low} to {high}")
                return False

        self.__target = list(positions)
        return True

    # AUTO GET STATUS
    @staticmethod
    def __limit(value: float, limits: tuple) -> float:
        """
        Return value inside [min, max]
        """
        return min(max(value, limits[0]), limits[1])

    @runInThread
    def __updateStatus(self):
        last = time()

        while True:
            now = time()
            dt = now - last
            last = now

            for i in range(ROBOT_JOINT_STATES.NUM_JOINTS):
                current = self.joint_state[i]
                target = self.__target[i]
                speed = ROBOT_SPEED.JOINTS_ORDER[i]
                limits = ROBOT_LIMIT.JOINTS_ORDER[i]
                step = min(dt * speed, abs(target - current))
                if target > current:
                    current += step
                elif target < current:
                    current -= step
                self.joint_state[i] = self.__limit(current, limits)

            print("JOINTS:", self.joint_state, self.__target)
            sleep(0.05)
