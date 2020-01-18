#!/usr/bin/env python3

# Copyright (c) 2017-2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.

import wpilib
import wheel_of_fortune.WheelOfFortune

class MyRobot(wpilib.TimedRobot):
	"""
		This is a simple example to show the values that can be read from the REV
		Color Sensor V3
	"""

	def robotInit(self):
		# Initialise components
		wheel = WheelOfFortune()


if __name__ == "__main__":
	wpilib.run(MyRobot)