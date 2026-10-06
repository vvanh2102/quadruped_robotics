#!/usr/bin/python3
import math
import numpy as np

from vanh_msgs.msg import ManualControl
from geometry_msgs.msg import Point32
from vanh_gait.kinematics import Kinematics

class PARAM_GAIT:
    STEP_HEIGHT  = 0.02     # meters
    STEP_LENGTH  = 0.03     # Length of the step ,meters
    STAND_HEIGHT = 0.2      # Height below the hip ,meters
    SWING_DURATION = 0.5    # Time to swing one leg on cycle 
    FORWARD_ORDER  = ['BL', 'FL', 'BR', 'FR']
    BACKWARD_ORDER = ['FR', 'BR', 'FL', 'BL']

class GAIT_STATE:
    def __init__(self):
        self.__kinematics = Kinematics()

        self.__velocity = np.zeros(3)
        self.__step_height = PARAM_GAIT.STEP_HEIGHT
        self.__step_length = PARAM_GAIT.STEP_LENGTH
        self.__stand_height = PARAM_GAIT.STAND_HEIGHT
        self.__swing_duration = PARAM_GAIT.SWING_DURATION

        self.__request_mission = ManualControl.STOP
        self.__current_mission = ManualControl.STOP
        self.__cycle_time = 0.0

    # METHOD
    def __islimitStepLength(self,start_position, end_position):
        """
        Limit the step length based on requested step_length.
        """
        return True
        
    def __FuncGeometry_path(self, x_time , start_position , end_position):
        """
        Func the geometry path of the foot in the trunk frame.
        """
        x_start = start_position.x
        x_end = end_position.x
        z_time = start_position.z + 4 * self.__step_height * (x_time - x_start) * (x_end - x_time) / (x_end - x_start) ** 2
        
        position = Point32()
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
            Point32 containing the foot position in the trunk frame.
        """
        x_time = self.__timingLaw(t , duration , start_position.x , end_position.x)
        return self.__FuncGeometry_path(x_time , start_position , end_position)

    def __getOneFootTrajectory(self, leg_name, t , duration , direction = 1):
        """
        Calculate one swing starting from the standing position.
        ```
        Parameters:
            leg_name: name of the leg
            t: elapsed swing time
            duration: total time of the swing
            direction: 1 for forward, -1 for backward
        Return: 
            Point32 in the trunk frame
        """
        standing_position = self.__kinematics.standing_target_onefoot(leg_name , self.__stand_height)
        
        start_position = Point32(
            x=float(standing_position[0]),
            y=float(standing_position[1]),
            z=float(standing_position[2]),
        )

        end_position = Point32(
            x=start_position.x + direction * self.__step_length,
            y=start_position.y,
            z=start_position.z,
        )

        if not self.__islimitStepLength(start_position, end_position):
            raise ValueError("Step length exceeds the limit.")
        else:
            return self.__calTrajectory(t , duration , start_position , end_position)

    # MANUAL MISSION 
    def controlManual(self, action):
        """
        Check request and handle the manual mission.
        ```
        Parameters:
            action: the action to be executed
        Return:
            bool: True if command accepted, False otherwise
        """
        respone = ""

        if action == ManualControl.STOP:
            self.__request_mission = action
            respone = "Action STOP executed."
        elif action == ManualControl.FORWARD:
            self.__request_mission = action
            respone = "Action FORWARD executed."
        elif action == ManualControl.BACKWARD:
            self.__request_mission = action
            respone = "Action BACKWARD executed."
        elif action == ManualControl.MOVE_LEFT:
            self.__request_mission = action
            respone = "Action MOVE_LEFT executed."
        elif action == ManualControl.MOVE_RIGHT:
            self.__request_mission = action
            respone = "Action MOVE_RIGHT executed."
        elif action == ManualControl.TURN_LEFT:     
            self.__request_mission = action
            respone = "Action TURN_LEFT executed."
        elif action == ManualControl.TURN_RIGHT:
            self.__request_mission = action
            respone = "Action TURN_RIGHT executed."
        elif action == ManualControl.CROUCH:
            self.__request_mission = action
            respone = "Action CROUCH executed."
        elif action == ManualControl.STAND:
            self.__request_mission = action
            respone = "Action STAND executed."
        else:
            respone = "Invalid action requested."
        return respone

    def Run_time(self,dt):
        """
        Manager all of mission 
        """
        if self.__current_mission == ManualControl.STOP:
            self.__current_mission = self.__request_mission
            self.__cycle_time = 0.0
        else:
            self.__cycle_time += dt
            cycle_duration = len(PARAM_GAIT.BACKWARD_ORDER) * self.__swing_duration
            
            if self.__cycle_time >= cycle_duration:
                self.__cycle_time %= cycle_duration
                self.__current_mission = self.__request_mission
            else:
                if self.__current_mission == ManualControl.STOP:
                    self.__cycle_time = 0.0
                    return self.__STOP()
                elif self.__current_mission == ManualControl.FORWARD:
                    return self.__FORWARD_OR_BACKWARD(direction=1 , cycle_time=self.__cycle_time)
                elif self.__current_mission == ManualControl.BACKWARD:
                    return self.__FORWARD_OR_BACKWARD(direction=-1, cycle_time=self.__cycle_time)
                elif self.__current_mission == ManualControl.MOVE_LEFT:
                    return self.__MOVE_LEFT_OR_MOVE_RIGHT()
                elif self.__current_mission == ManualControl.MOVE_RIGHT:
                    return self.__MOVE_LEFT_OR_MOVE_RIGHT()
                                            
                else:
                    return self.__STOP()
                                      
    def __STOP(self):
        """
        Handle the stop action.
        """
        targets = {}
        for leg_name in self.__kinematics.LEGS:
            targets[leg_name] = (self.__kinematics.standing_target_onefoot(leg_name,self.__stand_height))
        return targets

    def __FORWARD_OR_BACKWARD(self, direction , t):
        """
        Handle the forward action or backward action.
        """
        targets = {}
        if direction == 1:
            walk_order = PARAM_GAIT.FORWARD_ORDER
        else:
            walk_order = PARAM_GAIT.BACKWARD_ORDER

        swing_index = min(int(t / self.__swing_duration), len(walk_order) - 1)
        swing_time  = t - swing_index * self.__swing_duration
        swing_progress = self.__timingLaw(swing_time , self.__swing_duration , 0.0 , 1.0)
        trunk_offset_x = direction * self.__step_length * (swing_index + swing_progress) / len(walk_order)

        for leg_index, leg_name in enumerate(walk_order):
            if leg_index < swing_index:            
                foot_offset_x = direction * self.__step_length
                foot_lift = 0.0
            elif leg_index == swing_index:
                foot_offset_x = direction * self.__step_length * swing_progress
                foot_lift = 4.0 * self.__step_height * swing_progress * (1.0 - swing_progress)
            else:
                foot_offset_x = 0.0
                foot_lift = 0.0
            standing = self.__kinematics.standing_target_onefoot(leg_name, self.__stand_height)
            targets[leg_name] = standing + np.array([foot_offset_x - trunk_offset_x, 0.0, foot_lift])
            # targets[leg_name] = Point32(
            #     x=float(standing[0] + foot_offset_x - trunk_offset_x),
            #     y=float(standing[1]),
            #     z=float(standing[2] + foot_lift),
            # )
        return targets

    def __MOVE_LEFT_OR_MOVE_RIGHT(self , direction , t):
        """
        Handle the move left action or move right action.
        """

    def __TURN_LEFT_OR_TURN_RIGHT(self , direction , t):
        """
        Handle the turn left action or turn right action.
        """

    def __CROUCH(self):
        """
        Handle the crouch action.
        """

    def __STAND(self):
        """
        Handle the stand action.
        """

        