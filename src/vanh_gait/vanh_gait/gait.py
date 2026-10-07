#!/usr/bin/python3
import math
import numpy as np

from vanh_msgs.msg import ManualControl
from geometry_msgs.msg import Point32
from vanh_gait.kinematics import Kinematics

class PARAM_GAIT:
    # The constant displacement 
    STEP_HEIGHT  = 0.05     
    STEP_LENGTH  = 0.08
    SIDE_STEP_LENGTH = 0.03     
    STAND_HEIGHT = 0.17   
    TURN_ANGLE = math.radians(20.0)  

    # Time to swing one leg on cycle
    SWING_DURATION = 0.5     

    # Joint order
    FORWARD_ORDER    = ['BL', 'FL', 'BR', 'FR']
    BACKWARD_ORDER   = ['FR', 'BR', 'FL', 'BL']
    LEFT_ORDER       = ['BL', 'FL', 'BR', 'FR']
    RIGHT_ORDER      = ['BR', 'FR', 'BL', 'FL']
    TURN_LEFT_ORDER  = ['FR', 'FL', 'BL', 'BR']
    TURN_RIGHT_ORDER = ['FL', 'FR', 'BR', 'BL']

class GAIT_STATE:
    def __init__(self):
        self.__kinematics = Kinematics()

        self.__velocity = np.zeros(3)
        self.__step_height = PARAM_GAIT.STEP_HEIGHT
        self.__step_length = PARAM_GAIT.STEP_LENGTH
        self.__stand_height = PARAM_GAIT.STAND_HEIGHT
        self.__side_step_length = PARAM_GAIT.SIDE_STEP_LENGTH
        self.__turn_angle = PARAM_GAIT.TURN_ANGLE

        self.__swing_duration = PARAM_GAIT.SWING_DURATION

        self.__request_mission = ManualControl.STOP
        self.__current_mission = ManualControl.STOP
        self.__cycle_time = 0.0

    # METHOD
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

    def __getOneFootTrajectory(self, t, duration, start_position, end_position):
        """
        Apply the existing X-Z trajectory along a horizontal step.

        start_position, end_position: NumPy arrays at the same height
        Return: NumPy foot position in the input reference frame
        """
        displacement = end_position - start_position
        step_distance = float(np.linalg.norm(displacement[:2]))
        if step_distance == 0.0:
            return start_position.copy()

        path_start = Point32(x=0.0, y=0.0, z=0.0)
        path_end = Point32(x=step_distance, y=0.0, z=0.0)
        path_position = self.__calTrajectory(t, duration, path_start, path_end)
        progress = path_position.x / path_end.x
        foot_position = start_position + progress * displacement
        foot_position[2] += path_position.z

        return foot_position

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
        cycle_duration = len(PARAM_GAIT.BACKWARD_ORDER) * self.__swing_duration

        if self.__current_mission == ManualControl.STOP:
            self.__current_mission = self.__request_mission
            self.__cycle_time = 0.0
        else:
            self.__cycle_time += dt    
 
        if self.__cycle_time >= cycle_duration:
            self.__cycle_time %= cycle_duration
            self.__current_mission = self.__request_mission
    
        if self.__current_mission == ManualControl.STOP:
            self.__cycle_time = 0.0
            return self.__STOP()
        elif self.__current_mission == ManualControl.FORWARD:
            return self.__FORWARD_OR_BACKWARD(direction = 1 , t = self.__cycle_time)
        elif self.__current_mission == ManualControl.BACKWARD:
            return self.__FORWARD_OR_BACKWARD(direction = -1, t = self.__cycle_time)
        elif self.__current_mission == ManualControl.MOVE_LEFT:
            return self.__MOVE_LEFT_OR_MOVE_RIGHT(direction = 2, t = self.__cycle_time)
        elif self.__current_mission == ManualControl.MOVE_RIGHT:
            return self.__MOVE_LEFT_OR_MOVE_RIGHT(direction = -2, t = self.__cycle_time)
        elif self.__current_mission == ManualControl.TURN_LEFT:
            return self.__TURN_LEFT_OR_TURN_RIGHT(direction = 3 , t = self.__cycle_time)
        elif self.__current_mission == ManualControl.TURN_RIGHT:
            return self.__TURN_LEFT_OR_TURN_RIGHT(direction = -3 , t = self.__cycle_time)
        elif self.__current_mission == ManualControl.CROUCH:
            return self.__CROUCH()
        elif self.__current_mission == ManualControl.STAND:
            return self.__STAND()
                                            
    # WALK COORDINATION
    def __walk(self, t , walk_order , step_displacement , turn_angle = 0.0):
        """
        Coordinate four feet for one walking cycle.

        step_displacement: planned trunk translation per cycle [m]
        turn_angle: planned trunk yaw change per cycle [rad]

        Plan positions in a fixed frame matching the trunk frame
        at the cycle start, then convert to the moving trunk frame.
        """
        targets = {}
        swing_index = min(int(t / self.__swing_duration),len(walk_order) - 1)
        swing_time = t - swing_index * self.__swing_duration
        swing_progress = self.__timingLaw(swing_time, self.__swing_duration, 0.0, 1.0)

        cycle_progress = (swing_index + swing_progress) / len(walk_order)
        trunk_displacement = step_displacement * cycle_progress
        trunk_yaw = turn_angle * cycle_progress

        landing_rotation = self.__kinematics.rot_z(turn_angle)
        ground_to_trunk_rotation = self.__kinematics.rot_z(-trunk_yaw)

        for leg_index, leg_name in enumerate(walk_order):
            start_position = self.__kinematics.standing_target_onefoot(leg_name,self.__stand_height)
            end_position = landing_rotation @ start_position + step_displacement

            if leg_index < swing_index:
                foot_position = end_position
            elif leg_index == swing_index:
                foot_position = self.__getOneFootTrajectory(swing_time, self.__swing_duration, start_position, end_position)
            else:
                foot_position = start_position
            # p_trunk = Rz(-trunk_yaw) @ (p_fixed - trunk_translation)
            targets[leg_name] = ground_to_trunk_rotation @ (foot_position - trunk_displacement)
        return targets

    # MISSIONS
    def __STOP(self):
        """
        Return the neutral standing targets.
        """
        targets = {}
        for leg_name in self.__kinematics.LEGS:
            targets[leg_name] = (self.__kinematics.standing_target_onefoot(leg_name,self.__stand_height,))
        return targets

    def __FORWARD_OR_BACKWARD(self, direction, t):
        """
        Walk along X.

        direction: +1 forward, -1 backward
        """
        if direction == 1:
            walk_order = PARAM_GAIT.FORWARD_ORDER
        else:
            walk_order = PARAM_GAIT.BACKWARD_ORDER

        step_displacement = np.array([direction * self.__step_length, 0.0, 0.0])
        return self.__walk(t, walk_order, step_displacement)

    def __MOVE_LEFT_OR_MOVE_RIGHT(self, direction, t):
        """
        Walk along Y.

        direction: +2 left, -2 right
        """
        if direction == 2:
            walk_order = PARAM_GAIT.LEFT_ORDER
        else:
            walk_order = PARAM_GAIT.RIGHT_ORDER

        step_displacement = np.array([0.0,direction * self.__side_step_length,0.0])
        return self.__walk(t, walk_order, step_displacement)

    def __TURN_LEFT_OR_TURN_RIGHT(self, direction, t):
        """
        Turn around the trunk Z axis without planned translation.

        direction: +3 left, -3 right
        """
        if direction == 3:
            walk_order = PARAM_GAIT.TURN_LEFT_ORDER
            turn_angle = 1 * self.__turn_angle
        else:
            walk_order = PARAM_GAIT.TURN_RIGHT_ORDER
            turn_angle = -1 * self.__turn_angle

        return self.__walk(t, walk_order, np.zeros(3),turn_angle)
        