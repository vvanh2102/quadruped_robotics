#!/usr/bin/python3
from time import monotonic

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from vanh_msgs.msg import JointState, ManualControl
from vanh_gait.kinematics import Kinematics
from vanh_gait.gait import Gait


class Gait_Node(Node):
    RATE_HZ = 20.0
    MANUAL_TIMEOUT = 0.5

    STAND_HEIGHT = Gait.STAND_HEIGHT
    CROUCH_HEIGHT = 0.12

    SPEED_X = 0.02
    SPEED_Y = 0.01
    TURN_RATE = 0.1

    def __init__(self):
        super().__init__("gait_node")

        self.__kinematics = Kinematics()
        self.__gait = Gait(self.__kinematics)

        self.__actions = set()
        self.__height = self.STAND_HEIGHT
        self.__fault = False

        self.__last_manual = monotonic()
        self.__last_update = monotonic()

        # Publisher
        self.__cmd_pub = self.create_publisher(
            JointState,
            "joint_command",
            3,
        )

        # Subscriber
        self.__manual_sub = self.create_subscription(
            ManualControl,
            "manual",
            self.__onManual,
            3,
        )

        # Timer
        self.__timer = self.create_timer(
            1.0 / self.RATE_HZ,
            self.__update,
        )

    # CALLBACK
    def __onManual(self, msg):
        """
        Receive body-level manual commands.

        Opposite movement commands cancel each other.
        STOP takes priority over movement and posture.
        """
        actions = set(msg.actions)
        self.__last_manual = monotonic()

        allowed = {
            ManualControl.STOP,
            ManualControl.FORWARD,
            ManualControl.BACKWARD,
            ManualControl.MOVE_LEFT,
            ManualControl.MOVE_RIGHT,
            ManualControl.TURN_LEFT,
            ManualControl.TURN_RIGHT,
            ManualControl.CROUCH,
            ManualControl.STAND,
        }

        if not actions.issubset(allowed):
            self.__stop()
            self.get_logger().warning("Unknown manual action")
            return

        if ManualControl.STOP in actions:
            self.__stop()
            return

        self.__actions = actions

        crouch = ManualControl.CROUCH in actions
        stand = ManualControl.STAND in actions

        # Keep the previous height if posture commands conflict.
        if crouch and stand:
            return

        if crouch:
            self.__height = self.CROUCH_HEIGHT
        elif stand:
            self.__height = self.STAND_HEIGHT

    def __update(self):
        """Generate foot targets and publish joint commands."""
        now = monotonic()

        dt = now - self.__last_update
        self.__last_update = now
        dt = min(max(dt, 0.0), 0.1)

        if self.__fault:
            return

        velocity = self.__getVelocity(now)

        targets = self.__gait.update(
            dt,
            velocity,
            self.__height,
        )

        self.__publish(targets)

    # METHOD
    def __stop(self):
        """Request a smooth walking stop and hold the current height."""
        self.__actions = {ManualControl.STOP}
        self.__height = self.__gait.getHeight()

    def __getVelocity(self, now):
        """Convert manual actions to vx, vy and yaw rate."""
        if now - self.__last_manual > self.MANUAL_TIMEOUT:
            return np.zeros(3)

        actions = self.__actions

        if ManualControl.STOP in actions:
            return np.zeros(3)

        forward = (
            int(ManualControl.FORWARD in actions)
            - int(ManualControl.BACKWARD in actions)
        )

        left = (
            int(ManualControl.MOVE_LEFT in actions)
            - int(ManualControl.MOVE_RIGHT in actions)
        )

        turn = (
            int(ManualControl.TURN_LEFT in actions)
            - int(ManualControl.TURN_RIGHT in actions)
        )

        return np.array([
            forward * self.SPEED_X,
            left * self.SPEED_Y,
            turn * self.TURN_RATE,
        ])

    def __publish(self, targets):
        """Publish only when all four IK solutions are valid."""
        positions = []

        for leg_name, target in zip(
            self.__kinematics.LEGS,
            targets,
        ):
            angles = self.__kinematics.inverse_kinematics(
                leg_name,
                target,
            )

            if angles is None:
                self.__fault = True

                self.get_logger().error(
                    f"IK failed: {leg_name}, "
                    f"target={np.round(target, 5).tolist()}. "
                    "Check the trajectory and restart the node."
                )
                return

            positions.extend(angles)

        msg = JointState()
        msg.positions = positions

        self.__cmd_pub.publish(msg)

    # ENTRY POINT
    @staticmethod
    def main(args=None):
        """Start the ROS node."""
        rclpy.init(args=args)
        node = None

        try:
            node = Gait_Node()
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            if node is not None:
                node.destroy_node()

            rclpy.try_shutdown()