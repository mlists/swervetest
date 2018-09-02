#!/usr/bin/env python3
import math
import numpy as np

import ctre
import magicbot
import wpilib
from pyswervedrive.swervechassis import SwerveChassis
from pyswervedrive.swervemodule import SwerveModule
from utilities.imu import IMU
from utilities.functions import rescale_js, constrain_angle
from robotpy_ext.misc.looptimer import LoopTimer
from networktables import NetworkTables


class Robot(magicbot.MagicRobot):
    # Add magicbot components here using variable annotations.
    # Any components that directly actuate motors should be declared after
    # any higher-level components (automations) that depend on them.

    chassis: SwerveChassis

    module_drive_free_speed: float = 7800.  # encoder ticks / 100 ms

    def createObjects(self):
        """Create non-components here."""

        self.module_a = SwerveModule(  # top left module
            # "a", steer_talon=ctre.TalonSRX(48), drive_talon=ctre.TalonSRX(49),
            "a", steer_talon=ctre.TalonSRX(48), drive_talon=ctre.TalonSRX(41),
            x_pos=0.25, y_pos=0.31,
            drive_free_speed=Robot.module_drive_free_speed)
        self.module_b = SwerveModule(  # bottom left module
            "b", steer_talon=ctre.TalonSRX(58), drive_talon=ctre.TalonSRX(51),
            x_pos=-0.25, y_pos=0.31,
            drive_free_speed=Robot.module_drive_free_speed)
        self.module_c = SwerveModule(  # bottom right module
            "c", steer_talon=ctre.TalonSRX(52), drive_talon=ctre.TalonSRX(53),
            x_pos=-0.25, y_pos=-0.31,
            drive_free_speed=Robot.module_drive_free_speed)
        self.module_d = SwerveModule(  # top right module
            "d", steer_talon=ctre.TalonSRX(42), drive_talon=ctre.TalonSRX(43),
            x_pos=0.25, y_pos=-0.31,
            drive_free_speed=Robot.module_drive_free_speed)

        # create the imu object
        self.imu = IMU('navx')

        self.sd = NetworkTables.getTable("SmartDashboard")

        # boilerplate setup for the joystick
        self.joystick = wpilib.Joystick(0)
        self.gamepad = wpilib.XboxController(1)

        self.spin_rate = 1.5

    def teleopInit(self):
        '''Called when teleop starts; optional'''
        self.chassis.set_inputs(0, 0, 0)
        self.loop_timer = LoopTimer(self.logger)

    def teleopPeriodic(self):
        """
        Process inputs from the driver station here.

        This is run each iteration of the control loop before magicbot components are executed.
        """

        if self.joystick.getRawButtonPressed(10):
            self.imu.resetHeading()
            self.chassis.set_heading_sp(0)

        throttle = (1-self.joystick.getThrottle())/2

        # this is where the joystick inputs get converted to numbers that are sent
        # to the chassis component. we rescale them using the rescale_js function,
        # in order to make their response exponential, and to set a dead zone -
        # which just means if it is under a certain value a 0 will be sent
        # TODO: Tune these constants for whatever robot they are on
        joystick_vx = -rescale_js(self.joystick.getY(), deadzone=0.1, exponential=1.5, rate=4*throttle)
        joystick_vy = -rescale_js(self.joystick.getX(), deadzone=0.1, exponential=1.5, rate=4*throttle)
        joystick_vz = -rescale_js(self.joystick.getZ(), deadzone=0.2, exponential=20.0, rate=self.spin_rate)
        joystick_hat = self.joystick.getPOV()

        if joystick_vx or joystick_vy or joystick_vz:
            self.chassis.set_inputs(joystick_vx, joystick_vy, joystick_vz,
                                    field_oriented=not self.joystick.getRawButton(6))
        elif self.gamepad.getStickButton(self.gamepad.Hand.kLeft):
            # TODO Tune these constants for the gamepad.
            gamepad_vx = -rescale_js(self.gamepad.getY(self.gamepad.Hand.kRight), deadzone=0.1, exponential=1.5, rate=0.5)
            gamepad_vy = -rescale_js(self.gamepad.getX(self.gamepad.Hand.kRight), deadzone=0.1, exponential=1.5, rate=0.5)

            self.chassis.set_inputs(gamepad_vx, gamepad_vy, 0, field_oriented=True)
        else:
            self.chassis.set_inputs(0, 0, 0)

        if joystick_hat != -1:
            constrained_angle = -constrain_angle(math.radians(joystick_hat))
            self.chassis.set_heading_sp(constrained_angle)

    def testPeriodic(self):
        if self.gamepad.getStartButtonPressed():
            self.module_a.store_steer_offsets()
            self.module_b.store_steer_offsets()
            self.module_c.store_steer_offsets()
            self.module_d.store_steer_offsets()

    def robotPeriodic(self):
        # super().robotPeriodic()
        self.sd.putNumber("imu_heading", self.imu.getAngle())


if __name__ == '__main__':
    wpilib.run(Robot)
