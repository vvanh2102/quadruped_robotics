#!/usr/bin/python3
import math
import numpy as np

from vanh_msgs.msg import ManualControl
from vanh_msgs.msg import ManualControl, Geometry
from vanh_gait.kinematics import Kinematics

class PARAM_GAIT:
    STEP_HEIGHT  = 0.02   # meters
    STEP_LENGTH  = 0.03    # Length of the step ,meters
    STAND_HEIGHT = 0.2    # Height below the hip ,meters
    

class GAIT_STATE:
    def __init__(self):
        self.__kinematics = Kinematics()

        self.__velocity = np.zeros(3)
        self.__step_height = PARAM_GAIT.STEP_HEIGHT
        self.__step_length = PARAM_GAIT.STEP_LENGTH
        self.__stand_height = PARAM_GAIT.STAND_HEIGHT

    # METHOD
    def __limitStepLength(self,start_position, end_position):
        """
        Limit the step length based on the requested velocity.
        """
        
        return True
        
    def __FuncGeometry_path(self, x_time , start_position , end_position):
        """
        Func the geometry path of the foot in the trunk frame.
        """
        x_start = start_position.x
        x_end = end_position.x
        z_time = start_position.z + 4 * self.__step_height * (x_time - x_start) * (x_time - x_end) / (x_end - x_start) ** 2
        
        position = Geometry()
        position.x = float(x_time)
        position.y = float(start_position.y)
        position.z = float(z_time)
        return position

    def __timingLaw(self, t , duration , x_start , x_end):
        """
        Calculate the timing law of the foot in the trunk frame.
        ```
        t : elapsed swing time
        duration : total time
        x_start, x_end : start and end positions of the foot in the trunk frame 
        """
        x_time = x_start + (x_end - x_start) * (3.0*(t/duration)**2 - 2.0*(t/duration)**3)
        return x_time

    def __calTrajectory(self,t , duration , start_position , end_position):
        """
        Combine the timing law and geometry path 
        ```
        Return:
            Geometry containing the foot position in the trunk frame.
        """
        x_time = self.__timingLaw(t , duration , start_position.x , end_position.x)
        return self.__FuncGeometry_path(x_time , start_position , end_position)

    def __calFootTrajectory(self, leg_name, t , duration , direction = 1):
        """
        Calculate one swing starting from the standing position.
        ```
        Parameters:
            leg_name: name of the leg
            t: elapsed swing time
            duration: total time of the swing
            direction: 1 for forward, -1 for backward
        Return: 
            Geometry in the trunk frame
        """
        standing_position = self.__kinematics.standing_target_onefoot(leg_name , self.__stand_height)
        
        start_position = Geometry(
            x=float(standing_position[0]),
            y=float(standing_position[1]),
            z=float(standing_position[2]),
        )

        end_position = Geometry(
            x=start_position.x + direction * self.__step_length,
            y=start_position.y,
            z=start_position.z,
        )

        return self.__calTrajectory(t , duration , start_position , end_position)