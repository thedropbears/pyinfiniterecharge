#!/usr/bin/env python3

# Copyright (c) 2017-2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.

import wpilib
import rev
import rev.color

import magicbot

from controllers.shooter import ShooterController
from controllers.spinner import SpinnerController
from components.indexer import Indexer
from components.shooter import Shooter
from components.spinner import Spinner


class MyRobot(magicbot.MagicRobot):
    shooter_controller: ShooterController
    spinner_controller: SpinnerController
    indexer: Indexer
    shooter: Shooter
    spinner: Spinner

    def createObjects(self):
        """Robot initialization function"""
        # object that handles basic drive operations

        self.shooter_outer_motor = rev.CANSparkMax(3, rev.MotorType.kBrushless)
        self.shooter_centre_motor = rev.CANSparkMax(2, rev.MotorType.kBrushless)

        self.loading_piston = wpilib.Solenoid(0)

        self.indexer_motors = [wpilib.Spark(1), wpilib.Spark(0)]
        self.indexer_switches = [wpilib.DigitalInput(8), wpilib.DigitalInput(9)]

        self.spinner_motor = wpilib.Spark(2)
        self.spinner_solenoid = wpilib.Solenoid(2)
        self.colour_sensor = rev.color.ColorSensorV3(wpilib.I2C.Port.kOnboard)

        self.joysticks = joystick_handlers()

    def teleopInit(self):
        """Executed at the start of teleop mode"""

    def teleopPeriodic(self):
        """Executed every cycle"""

        self.joysticks.handle_indexer_inputs(joystick = self.joysticks.joystick_left, index_controller = self.indexer)
        self.joysticks.handle_spinner_inputs(joystick = self.joysticks.spinner_joystick, spinner_controller = self.spinner)
        self.joysticks.handle_shooter_inputs(joystick_left = self.joysticks.joystick_left, joystick_right = self.joysticks.joystick_right)


class joystick_handlers:
    def __init__(self):
        self.joystick_left = wpilib.Joystick(0)
        self.joystick_right = wpilib.Joystick(1)
        self.spinner_joystick = wpilib.Joystick(2)

    def handle_indexer_inputs(self, joystick = None, shooter_controller = None, indexer_controller = None):
        if joystick.getTrigger():
            shooter_controller.eject_cells()

        if joystick.getRawButtonPressed(3):
            shooter_controller.shoot_cells()

        if joystick.getRawButtonPressed(4):
            shooter_controller.intake_cells()

    def handle_spinner_inputs(self, joystick = None, spinner_controller = None):
        if joystick.getRawButtonPressed(7):
            spinner_controller.run(test=True, task="position")
            print(f"Position Control")

        if joystick.getRawButtonPressed(8):
            spinner_controller.run(test=True, task="rotation")
            print(f"Rotation Control")
    
    def handle_shooter_inputs(self, joystick_left = None, joystick_right = None, loading_piston = None, shooter = None):
        outer_throttle = ((-self.joystick_left.getThrottle() + 1) / 2) * 5000
        inner_throttle = -((-self.joystick_right.getThrottle() + 1) / 2) * 5000
        if loading_piston != None:
            shooter.set_motor_rpm(outer_throttle, inner_throttle)

        if joystick_left.getRawButtonPressed(11):
            loading_piston.startPulse()


if __name__ == "__main__":
    wpilib.run(MyRobot)
