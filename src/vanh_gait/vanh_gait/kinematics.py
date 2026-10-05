#!/usr0/bin/python3
import math
import os
import xml.etree.ElementTree as ET
import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory

class ANGLE_ORDER:
    HIP_ANGLE   = 0
    THIGH_ANGLE = 1
    CALF_ANGLE  = 2

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
    POSITION_TOLERANCE = 1e-7 

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
        axis_joint = None
        limits_joint = None
        joint = robot.find(f"joint[@name='{name}']")
        origin = joint.find("origin")
        axis_element = joint.find("axis")
        limit_element = joint.find("limit")

        if origin is not None:
            xyz = np.array(origin.get("xyz", "0 0 0").split(), dtype=float)
            rpy = np.array(origin.get("rpy", "0 0 0").split(), dtype=float)
        if axis_element is not None:
            axis_joint = np.array(axis_element.get("xyz", "1 0 0").split(),dtype=float)
        if limit_element is not None:
            limits_joint = (float(limit_element.get("lower")), float(limit_element.get("upper")))

        return {
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis_joint,
            "limits": limits_joint,
        }

    def load_model(self):
        """
        Load the geometry used by the simplified leg model.

        Return 
        -----------
        hip_position_in_trunk: np.ndarray
            The position of the hip joint in the trunk frame [x, y, z]
        thigh_position_in_hip: np.ndarray
            The position of the thigh joint in the hip frame [x, y, z]
        calf_position_in_thigh: np.ndarray
            The position of the calf joint in the thigh frame [x, y, z]
        foot_position_in_calf: np.ndarray
            The position of the foot joint in the calf frame [x, y, z]
        hip_axis_sign: float
            The sign of the hip joint when rotating around the +X axis
        thigh_axis_sign: float
            The sign of the thigh joint when rotating around the +Y axis
        calf_axis_sign: float
            The sign of the calf joint when rotating around the +Y axis
        hip_origin_rotation: np.ndarray
            The rotation matrix of origin hip
        thigh_origin_rotation: np.ndarray
            The rotation matrix of origin thigh
        calf_origin_rotation: np.ndarray
            The rotation matrix of origin calf
        joint_angle_limits: np.ndarray
            The joint angle limits in radians [[hip_lower, hip_upper], [thigh_lower, thigh_upper], [calf_lower, calf_upper]]        
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
                "hip_position_in_trunk" : hip["xyz"],
                "thigh_position_in_hip" : thigh["xyz"],
                "calf_position_in_thigh": calf["xyz"],
                "foot_position_in_calf" : foot["xyz"],
                "hip_axis_sign"         : hip["axis"][0],
                "thigh_axis_sign"       : thigh["axis"][1],
                "calf_axis_sign"        : calf["axis"][1],
                "hip_origin_rotation"   : self.rpy_matrix(hip["rpy"]),
                "thigh_origin_rotation" : self.rpy_matrix(thigh["rpy"]),
                "calf_origin_rotation"  : self.rpy_matrix(calf["rpy"]),
                "joint_angle_limits"    : np.array([hip["limits"], thigh["limits"], calf["limits"],],dtype=float)
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
        hip_rotation_in_trunk   = leg["hip_origin_rotation"] @ self.rot_x(leg["hip_axis_sign"] * angles[ANGLE_ORDER.HIP_ANGLE])
        thigh_rotation_in_trunk = hip_rotation_in_trunk @ leg["thigh_origin_rotation"] @ self.rot_y(leg["thigh_axis_sign"] * angles[ANGLE_ORDER.THIGH_ANGLE])
        calf_rotation_in_trunk  = thigh_rotation_in_trunk @ leg["calf_origin_rotation"] @ self.rot_y(leg["calf_axis_sign"] * angles[ANGLE_ORDER.CALF_ANGLE])
        foot_position_in_trunk  = leg["hip_position_in_trunk"] + hip_rotation_in_trunk @ leg["thigh_position_in_hip"] + thigh_rotation_in_trunk @ leg["calf_position_in_thigh"] + calf_rotation_in_trunk @ leg["foot_position_in_calf"]
        return foot_position_in_trunk

    # IK
    def inverse_kinematics(self, leg_name, target_position):
        """
        Calculate joint angles for one foot.

        Args:
            leg_name: FL, FR, BL or BR
            target_position: foot position in the trunk frame, in meters

        Return:
            [hip, thigh, calf] in radians, or None if no solution is found
        """
        try:
            target_position = np.asarray(target_position, dtype=float)
            leg = self.__legs[leg_name]
            target_in_hip_frame = leg["hip_origin_rotation"].T @ (target_position - leg["hip_position_in_trunk"])
            angles = self.__CalculateJointAngles(leg, target_in_hip_frame)
            return self.__refineJointAngles(leg_name, target_position, angles)
        except Exception as e:
            print(f"Error in inverse_kinematics for leg {leg_name}: {e}")
            return None

    def __CalculateJointAngles(self, leg, target_in_hip_frame):
        """
        Calculate an analytical seed for the downward leg branch.

        The seed ignores thigh and calf origin rotations.
        Hip rotates around X; thigh and calf rotate around Y.

        Return:
            numpy array [hip, thigh, calf] in radians, or None
        """
        x, y, z = target_in_hip_frame
        hip_to_thigh  = leg["thigh_position_in_hip"]
        thigh_to_calf = leg["calf_position_in_thigh"]
        calf_to_foot  = leg["foot_position_in_calf"]
        side_offset   = hip_to_thigh[1]
        height_offset = hip_to_thigh[2]
        thigh_length  = -thigh_to_calf[0]
        calf_length   = math.hypot(calf_to_foot[0], calf_to_foot[2])
        calf_zero_direction = math.atan2(calf_to_foot[2], calf_to_foot[0])

        # Solve hip angle in the YZ plane.
        distance_squared    = y * y + z * z - side_offset * side_offset
        if distance_squared <= 0.0:
            return None
        else:
            down_distance       = math.sqrt(distance_squared)
            target_direction_yz = math.atan2(z, y)
            zero_direction_yz   = math.atan2(-down_distance, side_offset)
            hip_angle           = leg["hip_axis_sign"] * self.wrap_angle(target_direction_yz - zero_direction_yz)

        # Solve the bend angle between the two links.
        plane_x = x
        plane_z = -down_distance - height_offset
        cosine_bend = (plane_x**2 + plane_z**2 - thigh_length**2 - calf_length**2) / (2.0 * thigh_length * calf_length)
        if cosine_bend < -1.0 - 1e-9 or cosine_bend > 1.0 + 1e-9:
            return None
        else:
            bend_angle = math.acos(max(-1.0, min(1.0, cosine_bend)))

        # Solve the thigh and calf angles in the XZ plane.
        target_direction_xz = math.atan2(plane_z, plane_x)
        triangle_angle = math.atan2(calf_length * math.sin(bend_angle),
                                    thigh_length + calf_length * math.cos(bend_angle))
        thigh_direction = target_direction_xz - triangle_angle
        thigh_angle = leg["thigh_axis_sign"] * self.wrap_angle(math.pi - thigh_direction)
        calf_angle  = leg["calf_axis_sign"]  * self.wrap_angle(calf_zero_direction - math.pi - bend_angle)

        return np.array([hip_angle, thigh_angle, calf_angle], dtype=float)

    # IK CORRECTION
    def __refineJointAngles(self, leg_name, target_position_in_trunk, joint_angles):
        """
        Refine joint angles using the full forward kinematics.

        Args:
            leg_name: FL, FR, BL or BR
            target_position_in_trunk: desired foot position, in meters
            joint_angles: initial [hip, thigh, calf] angles, in radians

        Return:
            refined joint angles as a list, or None if refinement fails
        """
        joint_angle_limits = self.__legs[leg_name]["joint_angle_limits"]
        lower_angle_limits = joint_angle_limits[:, 0] + self.LIMIT_MARGIN
        upper_angle_limits = joint_angle_limits[:, 1] - self.LIMIT_MARGIN
        joint_angles = np.clip(
            joint_angles, 
            lower_angle_limits, 
            upper_angle_limits
        )

        for _ in range(20):
            foot_position_in_trunk = self.forward_kinematics(leg_name, joint_angles)
            position_error = (target_position_in_trunk - foot_position_in_trunk)
            position_error_norm = np.linalg.norm(position_error)

            if position_error_norm < self.POSITION_TOLERANCE:
                return joint_angles.tolist()
            else:
                position_jacobian = np.zeros((3, 3))
                angle_increment = 1e-6
                for joint_index in range(3):
                    perturbed_angles = joint_angles.copy()
                    perturbed_angles[joint_index] += angle_increment
                    perturbed_foot_position_in_trunk = self.forward_kinematics(leg_name, perturbed_angles)
                    position_jacobian[:, joint_index] = (perturbed_foot_position_in_trunk- foot_position_in_trunk) / angle_increment

                # Calculate a damped joint angle correction.
                angle_correction = position_jacobian.T @ np.linalg.solve(position_jacobian @ position_jacobian.T+ 1e-8 * np.eye(3),position_error)
                angle_correction = np.clip(angle_correction, -0.05, 0.05)
                correction_accepted = False

                # Try smaller corrections until the position error decreases.
                for correction_scale in [1.0, 0.5, 0.25, 0.125]:
                    candidate_angles = np.clip(
                        joint_angles + correction_scale * angle_correction,
                        lower_angle_limits,
                        upper_angle_limits,
                    )

                    candidate_foot_position_in_trunk = self.forward_kinematics(leg_name, candidate_angles)
                    candidate_position_error_norm = np.linalg.norm(target_position_in_trunk- candidate_foot_position_in_trunk)

                    if candidate_position_error_norm < position_error_norm:
                        joint_angles = candidate_angles
                        correction_accepted = True
                        break

                if not correction_accepted:
                    return None

        return None

    # STANDING TARGET
    def standing_target_onefoot(self, leg_name, height_below_hip):
        """
        Calculate a standing foot target in the trunk frame.

        Args:
            leg_name: FL, FR, BL or BR
            height_below_hip: vertical distance below the hip along
                the trunk Z axis, in meters

        Return:
            foot position [x, y, z] in the trunk frame, in meters
        """
        leg = self.__legs[leg_name]
        hip_position_in_trunk = leg["hip_position_in_trunk"]
        lateral_offset        = leg["thigh_position_in_hip"][1]
        foot_offset_from_hip_in_trunk = np.array([0.0,lateral_offset,-height_below_hip,])
        foot_position_in_trunk = (hip_position_in_trunk + foot_offset_from_hip_in_trunk)
        return foot_position_in_trunk