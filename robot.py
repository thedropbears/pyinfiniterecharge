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
from components.turret import Turret

class MyRobot(magicbot.MagicRobot):
    shooter_controller: ShooterController
    spinner_controller: SpinnerController
    indexer: Indexer
    shooter: Shooter
    spinner: Spinner

    turret: Turret
    
    def createObjects(self):
        """Robot initialization function"""
        # object that handles basic drive operations

        self.shooter_outer_motor = rev.CANSparkMax(3, rev.MotorType.kBrushless)
        self.shooter_centre_motor = rev.CANSparkMax(2, rev.MotorType.kBrushless)

        self.loading_piston = wpilib.Solenoid(0)

        self.indexer_motors = [wpilib.Spark(9), wpilib.Spark(8), wpilib.Spark(7)]
        self.indexer_switches = [
            wpilib.DigitalInput(9),
            wpilib.DigitalInput(8),
            wpilib.DigitalInput(7),
        ]

        self.spinner_motor = wpilib.Spark(2)
        self.spinner_solenoid = wpilib.Solenoid(2)
        self.colour_sensor = rev.color.ColorSensorV3(wpilib.I2C.Port.kOnboard)
        self.shooter_loading_piston = wpilib.DoubleSolenoid(0, 1)

<<<<<<< HEAD
        self.joysticks = joystick_handlers()

=======
        self.turret_motor = wpilib.Spark(3)
        self.turret_left_index = wpilib.DigitalInput (1)
        self.turret_centre_index = wpilib.DigitalInput (0)
        self.turret_right_index = wpilib.DigitalInput (2)
    
>>>>>>> origin/master
    def teleopInit(self):
        """Executed at the start of teleop mode"""

    def teleopPeriodic(self):
        """Executed every cycle"""

        self.joysticks.handle_indexer_inputs(joysticks = [self.joysticks.joystick_left, self.joysticks.joystick_right], shooter_controller = self.shooter_controller, shooter = self.shooter, loading_piston = self.loading_piston)
        self.joysticks.handle_spinner_inputs(joystick = self.joysticks.spinner_joystick, spinner_controller = self.spinner_controller)


class joystick_handlers:
    def __init__(self):
        self.joystick_left = wpilib.Joystick(0)
        self.joystick_right = wpilib.Joystick(1)
        self.spinner_joystick = wpilib.Joystick(2)

    def handle_indexer_inputs(self, joysticks = [], shooter_controller = None, shooter = None, loading_piston = None): #will eventually just take controllers but for the moment takes some components to test
        if joysticks[0].getTrigger():
            shooter_controller.eject_cells()

        if joysticks[0].getRawButtonPressed(3):
            shooter_controller.shoot_cells()

        self.handle_indexer_inputs(self.joystick_left)
        self.handle_spinner_inputs(self.spinner_joystick)

        self.handle_spinner_inputs(self.spinner_joystick)

        if joysticks[0].getRawButtonPressed(11):
            loading_piston.startPulse()

    def handle_spinner_inputs(self, joystick = None, spinner_controller = None):
        if joystick.getRawButtonPressed(7):
            spinner_controller.run(test=True, task="position")
            print(f"Position Control")

        if joystick.getRawButtonPressed(8):
            spinner_controller.run(test=True, task="rotation")
            print(f"Rotation Control")


if __name__ == "__main__":
    wpilib.run(MyRobot)
