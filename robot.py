#!/usr/bin/env python3

# Copyright (c) 2017-2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.

import wpilib
import rev

import magicbot

from components.shooter import Shooter

class MyRobot(magicbot.MagicRobot):
    shooter: Shooter

    
    def createObjects(self):
        """Robot initialization function"""

        # object that handles basic drive operations
        self.joystick_left = wpilib.Joystick(0)
        self.joystick_right = wpilib.Joystick(1)

        self.shooter_outer_motor = rev.CANSparkMax(3, rev.MotorType.kBrushless)
        self.shooter_centre_motor = rev.CANSparkMax(2, rev.MotorType.kBrushless) 

        

        self.loading_piston = wpilib.DoubleSolenoid(0, 1)

        
        

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        


    def teleopPeriodic(self):
        """Runs the motors with tank steering"""

        outer_throttle = ((-self.joystick_left.getThrottle() +1)/2)* 5000
        inner_throttle = -((-self.joystick_right.getThrottle() +1)/2)* 5000

        self.shooter.set_motor_rpm(outer_throttle,inner_throttle)

        wpilib.SmartDashboard.putNumber("outerError", self.shooter.get_outer_error())
        wpilib.SmartDashboard.putNumber("centreError", self.shooter.get_centre_error())

        wpilib.SmartDashboard.putNumber("outerVelocity", outer_throttle)
        wpilib.SmartDashboard.putNumber("centreVelocity", inner_throttle)

        if self.joystick_left.getRawButtonPressed(11):
            self.loading_piston.set(wpilib.DoubleSolenoid.Value.kForward)
        if self.joystick_left.getRawButtonPressed(12):
            self.loading_piston.set(wpilib.DoubleSolenoid.Value.kReverse)




if __name__ == "__main__":
    wpilib.run(MyRobot)
