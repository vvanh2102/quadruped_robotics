#!/usr/bin/python3
from time import monotonic
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from vanh_msgs.msg import JointState, ManualControl
from vanh_gait.kinematics import Kinematics
from vanh_gait.gait import PARAM_GAIT, GAIT_STATE

class Gait_Utils:
    RATE_HZ = 20.0


class Gait_Node(Node):

    def __init__(self):
        super().__init__("gait_node")
        self.__kinematics = Kinematics()
      
        self.__gait_state = GAIT_STATE()

        self.__last_update = monotonic()

        # Publisher
        self.__cmd_pub = self.create_publisher(JointState,"joint_command",3)

        # Subscriber
        self.__manual_sub = self.create_subscription(ManualControl,"manual",self.__onManual,3)

        # Timer
        self.__cmd_timer = self.create_timer(1.0 / Gait_Utils.RATE_HZ , self.__pubJointCommand)

    def __onManual(self,msg: ManualControl):
        """
        Callback function for manual control messages.
        ```
        int8 action

        int8 FRONT_LEFT_JOINT_1  = 0
        int8 FRONT_LEFT_JOINT_2  = 1
        int8 FRONT_LEFT_JOINT_3  = 2
        int8 FRONT_RIGHT_JOINT_1 = 3
        int8 FRONT_RIGHT_JOINT_2 = 4
        int8 FRONT_RIGHT_JOINT_3 = 5
        int8 BACK_LEFT_JOINT_1   = 6
        int8 BACK_LEFT_JOINT_2   = 7
        int8 BACK_LEFT_JOINT_3   = 8
        int8 BACK_RIGHT_JOINT_1  = 9
        int8 BACK_RIGHT_JOINT_2  = 10
        int8 BACK_RIGHT_JOINT_3  = 11

        int8 STOP = 0
        int8 FORWARD = 1
        int8 BACKWARD = -1
        int8 MOVE_LEFT = 2
        int8 MOVE_RIGHT = -2
        int8 TURN_LEFT = 3
        int8 TURN_RIGHT = -3
        int8 CROUCH = 4
        int8 STAND = 5
        """
        action = msg.action
        self.get_logger().info(f"CONTROL MANUAL:")
        self.get_logger().info(f"\t action: {action}")

        res = self.__gait_state.controlManual(action)
        self.get_logger().info(f"CONTROL MANUAL: Result ({res})")

    def __pubJointCommand(self):
        """
        Publish joint commands 
        """
        now = monotonic()
        dt = now - self.__last_update
        self.__last_update = now
        
        foot_targets = self.__gait_state.Run_time(dt)
        joint_angles = []
        for leg_name in self.__kinematics.LEGS:
            angles = self.__kinematics.inverse_kinematics(leg_name , foot_targets[leg_name])
            if angles is None:
                self.get_logger().error(f"IK failed for {leg_name}. Publishing stopped.")
                return None
            else:
                for angle in angles:
                    joint_angles.append(float(angle))

        msg = JointState()
        msg.positions = joint_angles
        self.__cmd_pub.publish(msg)
        self.get_logger().info(f"Pub:topic /joint_command",throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = Gait_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
