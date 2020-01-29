#!/usr/bin/env python3

# Copyright (c) 2017-2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.

import wpilib
import ctre
import rev
import rev.color

import magicbot

from controllers.shooter import ShooterController
from controllers.spinner import SpinnerController
from components.indexer import Indexer
from components.shooter import Shooter
from components.spinner import Spinner
from components.chassis import Chassis
from components.hang import Hang
from components.vision import Vision
from components.turret import Turret
from utilities.scale_value import scale_value


class MyRobot(magicbot.MagicRobot):
    shooter_controller: ShooterController
    spinner_controller: SpinnerController
    indexer: Indexer
    shooter: Shooter
    spinner: Spinner
    chassis: Chassis
    hang: Hang
    turret: Turret

    def createObjects(self):
        """Robot initialization function"""
        # object that handles basic drive operations
        self.joystick_left = wpilib.Joystick(0)
        self.joystick_right = wpilib.Joystick(1)
        self.spinner_joystick = wpilib.Joystick(2)
        self.turret_joystick = wpilib.Joystick(3)

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
        self.spinner_solenoid = wpilib.DoubleSolenoid(2, 3)
        self.colour_sensor = rev.color.ColorSensorV3(wpilib.I2C.Port.kOnboard)
        self.shooter_loading_piston = wpilib.DoubleSolenoid(0, 1)

        self.vision = Vision()

        self.turret_centre_index = wpilib.DigitalInput(0)
        self.turret_motor = ctre.WPI_TalonSRX(10)

        self.chassis_left_rear = rev.CANSparkMax(4, rev.MotorType.kBrushless)
        self.chassis_left_front = rev.CANSparkMax(5, rev.MotorType.kBrushless)
        self.chassis_right_rear = rev.CANSparkMax(6, rev.MotorType.kBrushless)
        self.chassis_right_front = rev.CANSparkMax(7, rev.MotorType.kBrushless)

    def teleopInit(self):
        """Executed at the start of teleop mode"""

    def teleopPeriodic(self):
        """Executed every cycle"""

        self.handle_spinner_inputs(self.spinner_joystick)
        self.handle_chassis_inputs(self.joystick_left)

        pov = self.turret_joystick.getPOV(0)
        about_five_degrees = 0.087  # radians
        if pov != -1:
            if pov < 180:
                self.turret.slew(about_five_degrees)
            else:
                self.turret.slew(-about_five_degrees)

        if self.joystick_left.getRawButtonPressed(7):
            if self.indexer.indexing:
                self.indexer.disable_indexing()
            else:
                self.indexer.enable_indexing()

        self.handle_spinner_inputs(self.spinner_joystick)
        self.handle_shooter_inputs(self.joystick_left)

    def handle_spinner_inputs(self, joystick):
        if joystick.getRawButtonPressed(7):
            self.spinner_controller.run(test=True, task="position")
            print(f"Spinner Running")
        if joystick.getRawButtonPressed(9):
            self.spinner.piston_up()
            print("Spinner Piston Up")
        if joystick.getRawButtonPressed(10):
            self.spinner.piston_down()
            print("Spinner Piston Down")
        if joystick.getRawButtonPressed(8):
            print(f"Detected Colour: {self.spinner_controller.get_current_colour()}")
            print(f"Distance: {self.spinner_controller.get_wheel_dist()}")

    def handle_chassis_inputs(self, joystick):
        scaled_throttle = scale_value(joystick.getThrottle(), 1, -1, 0, 1)
        vx = scale_value(joystick.getY(), 1, -1, -1, 1, 2) * scaled_throttle
        vz = scale_value(joystick.getX(), 1, -1, -1, 1, 2) * scaled_throttle
        self.chassis.drive(vx, vz)

    def handle_shooter_inputs(self, joystick: wpilib.Joystick):
        if joystick.getTriggerPressed():
            self.shooter_controller.driver_input(True)
        if joystick.getTriggerReleased():
            self.shooter_controller.driver_input(False)


if __name__ == "__main__":
    wpilib.run(MyRobot)
