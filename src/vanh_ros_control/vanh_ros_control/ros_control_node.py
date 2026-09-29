#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from vanh_msgs.msg import JointState, RobotInformation
from vanh_ros_control.fake_interface import Sim_Interface


class Ros_Control_Node(Node):
    def __init__(self) -> None:
        super().__init__('ros_control_node')
        self.__interface = Sim_Interface()

        # Publisher
        self.__info_pub = self.create_publisher(RobotInformation, 'robot_info', 3)
        self.__info_timer = self.create_timer(0.2, self.__publishInfo)

        # Subscriber
        self.__cmd_sub = self.create_subscription(JointState, 'joint_command', self.__onJointCommand, 3)

    def __publishInfo(self) -> None:
        """
        Publish robot information
        ```
        builtin_interfaces/Time stamp
        JointState joint
        Stm32Status stm32
        bool[] devices

        int8 SERVO_POWER = 0
        int8 LIDAR_POWER = 1
        ```
        """
        msg = RobotInformation()

        msg.joint.positions = self.__interface.joint_state
        msg.stm32.software_version = self.__interface.version
        msg.stm32.connection = self.__interface.connected()
        msg.stm32.errors = self.__interface.error
        msg.devices = self.__interface.device
        msg.stamp = self.get_clock().now().to_msg()

        self.__info_pub.publish(msg)

    def __onJointCommand(self, msg: JointState) -> None:
        """
        Callback when receive joint command from Gait + IK
        ```
        float32[] positions
        ```
        """
        self.__interface.controlManualJoint(msg.positions)


def main(args=None):
    rclpy.init(args=args)
    node = Ros_Control_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
