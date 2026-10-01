#!/usr/bin/python3
import math
import numpy as np

class Gait:
    PERIOD = 1.0
    STEP_HEIGHT = 0.01
    STAND_HEIGHT = 0.16
    SMOOTH_TIME = 0.3
    HEIGHT_SPEED = 0.02

    # FL, FR, BL, BR.
    PHASE_OFFSET = (0.0, 0.5, 0.5, 0.0)

    def __init__(self, kinematics):
        self.__kinematics = kinematics
        self.__phase = 0.0
        self.__velocity = np.zeros(3)
        self.__lift = 0.0
        self.__height = self.STAND_HEIGHT

    # METHOD
    def getHeight(self):
        """Return the current requested height below the hip."""
        return self.__height

    def update(self, dt, velocity, height):
        """
        Calculate four foot targets in the trunk frame.

        dt: elapsed time, in seconds
        velocity: [vx, vy, yaw_rate], in m/s and rad/s
        height: distance below the hip, in meters
        """
        dt = min(max(dt, 0.0), 0.1)
        velocity = np.asarray(velocity, dtype=float)

        # Smooth movement commands.
        alpha = 1.0 - math.exp(-dt / self.SMOOTH_TIME)
        moving = np.linalg.norm(velocity) > 1e-8

        self.__velocity += alpha * (
            velocity - self.__velocity
        )

        target_lift = 1.0 if moving else 0.0
        self.__lift += alpha * (target_lift - self.__lift)

        # Limit the speed of posture changes.
        self.__height += np.clip(
            height - self.__height,
            -self.HEIGHT_SPEED * dt,
            self.HEIGHT_SPEED * dt,
        )

        stopped = (
            not moving
            and np.linalg.norm(self.__velocity) < 1e-5
            and self.__lift < 1e-4
        )

        if stopped:
            self.__velocity[:] = 0.0
            self.__lift = 0.0
            self.__phase = 0.0
        else:
            self.__phase = (
                self.__phase + dt / self.PERIOD
            ) % 1.0

        vx, vy, turn = self.__velocity
        targets = []

        for leg_name, phase_offset in zip(
            self.__kinematics.LEGS,
            self.PHASE_OFFSET,
        ):
            home = self.__kinematics.home(
                leg_name,
                self.__height,
            )

            # Include rotation around the trunk origin.
            leg_vx = vx - turn * home[1]
            leg_vy = vy + turn * home[0]

            # Stance lasts half of the gait period.
            step_x = leg_vx * self.PERIOD / 2.0
            step_y = leg_vy * self.PERIOD / 2.0

            phase = (self.__phase + phase_offset) % 1.0

            offset = self.__footOffset(
                phase,
                step_x,
                step_y,
                self.__lift * self.STEP_HEIGHT,
            )

            targets.append(home + offset)

        return targets

    # HELPER
    @staticmethod
    def __footOffset(phase, step_x, step_y, lift):
        """
        Return the foot offset from its standing position.

        Position is continuous at phase boundaries.
        Velocity is not continuous in this simple trajectory.
        """
        if phase < 0.5:
            # Swing: move forward and lift the foot.
            progress = phase / 0.5

            return np.array([
                step_x * (progress - 0.5),
                step_y * (progress - 0.5),
                lift * math.sin(math.pi * progress),
            ])

        # Stance: move backward relative to the body.
        progress = (phase - 0.5) / 0.5

        return np.array([
            step_x * (0.5 - progress),
            step_y * (0.5 - progress),
            0.0,
        ])