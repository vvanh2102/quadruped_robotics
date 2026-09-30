#!/usr0/bin/python3
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory

class Kinematics:
    # LINK ORDER
    LEGS = ['FL', 'FR', 'BL', 'BR']

    # URDF NAME
    URDF_NAMES = {
        'FL': 'FL', 
        'FR': 'FR', 
        'BL': 'RL', 
        'BR': 'RR'
    }

    # LIMIT MARGIN
    LIMIT_MARGIN = 1e-6

    # INITIALIZATION
    def __init__(self) -> None:
        self.__legs = {}   
        self.__load_model_robot = self.load_model()
        
    # MATHEMTICAL
    @staticmethod
    def rot_x(angle):
        """
        Rotation matrix around X.
        """
        c = math.cos(angle)
        s = math.sin(angle)

        return np.array([
            [1,  0,  0],
            [0,  c, -s],
            [0,  s,  c],
        ])

    @staticmethod
    def rot_y(angle):
        """
        Rotation matrix around Y.
        """
        c = math.cos(angle)
        s = math.sin(angle)

        return np.array([
            [c,  0,  s],
            [0,  1,  0],
            [-s, 0,  c],
        ])

    @staticmethod
    def rot_z(angle):
        """
        Rotation matrix around Z.
        """
        c = math.cos(angle)
        s = math.sin(angle)

        return np.array([
            [c, -s,  0],
            [s,  c,  0],
            [0,  0,  1],
        ])

    @staticmethod
    def wrap_angle(angle):
        """
        Wrap an angle using its sine and cosine.
        """
        return math.atan2(math.sin(angle), math.cos(angle))

    @classmethod
    def rpy_matrix(cls, rpy):
        """
        URDF joint orientation: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
        """
        roll, pitch, yaw = rpy
        return cls.rot_z(yaw) @ cls.rot_y(pitch) @ cls.rot_x(roll)


    # UTILS
    @staticmethod
    def read_joint(robot, name):
        """
        Read the position, rotation, axis and limits of one joint.
        """
        xyz = np.zeros(3)
        rpy = np.zeros(3)
        axis = None
        limits = None
        joint = robot.find(f"joint[@name='{name}']")
        origin = joint.find("origin")
        axis_element = joint.find("axis")
        limit_element = joint.find("limit")

        if origin is not None:
            xyz = np.array(origin.get("xyz", "0 0 0").split(), dtype=float)
            rpy = np.array(origin.get("rpy", "0 0 0").split(), dtype=float)
        if axis_element is not None:
            axis = np.array(axis_element.get("xyz", "1 0 0").split(),dtype=float)
        if limit_element is not None:
            limits = (float(limit_element.get("lower")), float(limit_element.get("upper")))

        return {
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis,
            "limits": limits,
        }

    def load_model(self):
        """
        Load the geometry used by the simplified leg model.
        """
        path = os.path.join(get_package_share_directory('vanh_description'), 'config', 'vanh.urdf.xacro')
        robot = ET.fromstring(xacro.process_file(path).toxml())

        for leg_name in self.LEGS:
            prefix = self.URDF_NAMES[leg_name]
            hip    = self.read_joint(robot, prefix + "_hip_joint")
            thigh  = self.read_joint(robot, prefix + "_thigh_joint")
            calf   = self.read_joint(robot, prefix + "_calf_joint")
            foot   = self.read_joint(robot, prefix + "_foot_fixed")
            
            self.__legs[leg_name] = {
                "hip": hip["xyz"],
                "thigh": thigh["xyz"],
                "calf": calf["xyz"],
                "foot": foot["xyz"],
                "hip_sign": hip["axis"][0],
                "thigh_sign": thigh["axis"][1],
                "calf_sign": calf["axis"][1],
                "hip_rpy": self.rpy_matrix(hip["rpy"]),
                "thigh_rpy": self.rpy_matrix(thigh["rpy"]),
                "calf_rpy": self.rpy_matrix(calf["rpy"]),
                "limits": [
                    hip["limits"],
                    thigh["limits"],
                    calf["limits"],
                ],
            }

        return self.__legs

    # FK
    def forward_kinematics(self, leg_name, angles):
        """
        Calculate the position of the foot

        Arg:
            leg_name: str
                The name of the leg (FL, FR, BL, BR)
            angles: list
                The joint angles in radians [hip, thigh, calf]
        
        Return:
            foot_position: np.ndarray
                The position of the foot in the trunk frame [x, y, z]
        """
        leg = self.__load_model_robot[leg_name]
        angles = np.asarray(angles, dtype=float)
        hip_rotation   = leg["hip_rpy"] @ self.rot_x(leg["hip_sign"] * angles[0])
        thigh_rotation = hip_rotation @ leg["thigh_rpy"] @ self.rot_y(leg["thigh_sign"] * angles[1])
        calf_rotation  = thigh_rotation @ leg["calf_rpy"] @ self.rot_y(leg["calf_sign"] * angles[2])
        foot_position  = leg["hip"] + hip_rotation @ leg["thigh"] + thigh_rotation @ leg["calf"] + calf_rotation @ leg["foot"]
        return foot_position

    # IK
    def inverse_kinematics(self, leg_name, target):
        """
        Calculate joint angles for one foot.

        target: [x, y, z] in the trunk frame, in meters
        Return: [hip, thigh, calf] in radians, or None
        """
        target = np.asarray(target, dtype=float)

        if target.shape != (3,) or not np.isfinite(target).all():
            return None

        leg = self.__legs[leg_name]

        # Move the target into the hip joint frame.
        x, y, z = leg["hip_rpy"].T @ (target - leg["hip"])

        side = leg["thigh"][1]
        height = leg["thigh"][2]

        thigh_length = -leg["calf"][0]
        calf_length = math.hypot(
            leg["foot"][0], leg["foot"][2]
        )
        calf_zero = math.atan2(
            leg["foot"][2], leg["foot"][0]
        )

        # 1. Solve the hip angle.
        distance_squared = y * y + z * z - side * side

        if distance_squared <= 0.0:
            return None

        distance = math.sqrt(distance_squared)

        hip = leg["hip_sign"] * self.wrap_angle(
            math.atan2(z, y) - math.atan2(-distance, side)
        )

        # 2. Solve the thigh-calf triangle.
        plane_z = -distance - height

        cosine = (
            x * x + plane_z * plane_z
            - thigh_length**2 - calf_length**2
        ) / (2.0 * thigh_length * calf_length)

        if cosine < -1.0 - 1e-9 or cosine > 1.0 + 1e-9:
            return None

        knee = math.acos(max(-1.0, min(1.0, cosine)))

        direction = math.atan2(plane_z, x) - math.atan2(
            calf_length * math.sin(knee),
            thigh_length + calf_length * math.cos(knee),
        )

        # 3. Convert to URDF joint angles.
        thigh = leg["thigh_sign"] * self.wrap_angle(
            math.pi - direction
        )
        calf = leg["calf_sign"] * self.wrap_angle(
            calf_zero - math.pi - knee
        )

        angles = np.array([hip, thigh, calf])

        # 4. Correct the small error caused by internal origin rotations.
        return self.__refine_angles(leg_name, target, angles)

       # IK CORRECTION
    def __refine_angles(self, leg_name, target, angles):
        """Correct the analytical result using the full FK."""
        limits = self.__legs[leg_name]["limits"]
        lower = limits[:, 0] + self.LIMIT_MARGIN
        upper = limits[:, 1] - self.LIMIT_MARGIN

        if not np.isfinite(angles).all():
            return None

        angles = np.clip(angles, lower, upper)

        for _ in range(20):
            current = self.forward_kinematics(leg_name, angles)
            error = target - current
            error_size = np.linalg.norm(error)

            if error_size < self.POSITION_TOLERANCE:
                return angles.tolist()

            # Measure how each joint changes the foot position.
            jacobian = np.zeros((3, 3))
            small_angle = 1e-6

            for index in range(3):
                test_angles = angles.copy()
                test_angles[index] += small_angle

                test_position = self.forward_kinematics(
                    leg_name, test_angles
                )

                jacobian[:, index] = (
                    test_position - current
                ) / small_angle

            # Calculate a small correction with damping.
            step = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + 1e-8 * np.eye(3),
                error,
            )
            step = np.clip(step, -0.05, 0.05)

            improved = False

            for scale in [1.0, 0.5, 0.25, 0.125]:
                candidate = np.clip(
                    angles + scale * step,
                    lower,
                    upper,
                )

                candidate_position = self.forward_kinematics(
                    leg_name, candidate
                )

                if np.linalg.norm(target - candidate_position) < error_size:
                    angles = candidate
                    improved = True
                    break

            if not improved:
                return None

        return None

    # STANDING TARGET
    def home(self, leg_name, height):
        """
        Return a standing foot target in the trunk frame.

        height: vertical distance below the hip, in meters
        """
        leg = self.__legs[leg_name]

        return leg["hip"] + np.array([
            0.0,
            leg["thigh"][1],
            -height,
        ])

