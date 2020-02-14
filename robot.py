#!/usr/bin/env python3

# Copyright (c) 2017-2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.

import math

import ctre
import magicbot
import navx
import rev.color
import wpilib

from components.chassis import Chassis
from components.hang import Hang
from components.indexer import Indexer
from components.shooter import Shooter
from components.spinner import Spinner
from components.turret import Turret
from components.vision import Vision
from components.led_screen import LEDScreen
from controllers.shooter import ShooterController
from controllers.spinner import SpinnerController
from utilities import git
from utilities.scale_value import scale_value

GIT_COMMIT = git.describe()


class MyRobot(magicbot.MagicRobot):
    # List sensors which should collect data before controllers here.
    vision: Vision

    # List controllers (which require components) here.
    shooter_controller: ShooterController
    spinner_controller: SpinnerController

    # List components (which represent physical subsystems) here.
    chassis: Chassis
    hang: Hang
    indexer: Indexer
    shooter: Shooter
    spinner: Spinner
    turret: Turret
    led_screen: LEDScreen

    def createObjects(self):
        """Robot initialization function"""
        self.logger.info("pyinfiniterecharge %s", GIT_COMMIT)

        self.chassis_left_front = rev.CANSparkMax(5, rev.MotorType.kBrushless)
        self.chassis_left_rear = rev.CANSparkMax(4, rev.MotorType.kBrushless)
        self.chassis_right_front = rev.CANSparkMax(7, rev.MotorType.kBrushless)
        self.chassis_right_rear = rev.CANSparkMax(6, rev.MotorType.kBrushless)
        self.imu = navx.AHRS.create_spi(update_rate_hz=50)

        self.hang_winch_motor_master = ctre.WPI_TalonSRX(21)
        self.hang_winch_motor_slave = ctre.WPI_TalonSRX(22)
        self.hang_kracken_hook_latch = wpilib.DoubleSolenoid(4, 5)

        self.indexer_motors = [
            ctre.WPI_TalonSRX(11),
            ctre.WPI_TalonSRX(12),
            ctre.WPI_TalonSRX(13),
            ctre.WPI_TalonSRX(14),
        ]
        self.piston_switch = wpilib.DigitalInput(4)  # checks if injector retracted
        self.injector_switch = wpilib.DigitalInput(9)
        self.injector_master_motor = ctre.WPI_TalonSRX(3)
        self.injector_slave_motor = ctre.WPI_TalonSRX(43)
        self.intake_arm_piston = wpilib.Solenoid(1)
        self.intake_main_motor = ctre.WPI_TalonSRX(18)
        self.intake_left_motor = wpilib.Spark(5)
        self.intake_right_motor = wpilib.Spark(6)

        self.led_screen_led = wpilib.AddressableLED(0)
        self.loading_piston = wpilib.Solenoid(0)
        self.shooter_centre_motor = rev.CANSparkMax(3, rev.MotorType.kBrushless)
        self.shooter_outer_motor = rev.CANSparkMax(2, rev.MotorType.kBrushless)

        self.colour_sensor = rev.color.ColorSensorV3(wpilib.I2C.Port.kOnboard)
        self.spinner_motor = wpilib.Spark(2)
        self.spinner_solenoid = wpilib.DoubleSolenoid(2, 3)

        self.turret_centre_index = wpilib.DigitalInput(0)
        self.turret_motor = ctre.WPI_TalonSRX(10)

        # operator interface
        self.driver_joystick = wpilib.Joystick(0)

        self.MEMORY_CONSTANT = int(0.1 / self.control_loop_wait_time)
        # how long before data times out

    def teleopInit(self):
        """Executed at the start of teleop mode"""

    def teleopPeriodic(self):
        """Executed every cycle"""
        self.handle_indexer_inputs(self.driver_joystick)
        self.handle_chassis_inputs(self.driver_joystick)
        self.handle_spinner_inputs(self.driver_joystick)
        self.handle_shooter_inputs(self.driver_joystick)
        self.handle_hang_inputs(self.driver_joystick)

    def handle_indexer_inputs(self, joystick: wpilib.Joystick) -> None:
        if joystick.getRawButtonPressed(6):
            if self.indexer.intaking:
                self.indexer.disable_intaking()
            else:
                self.indexer.enable_intaking()

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
        vx = scale_value(joystick.getY(), 1, -1, -3, 3, 2) * scaled_throttle
        vz = scale_value(joystick.getX(), 1, -1, -3, 3, 2) * scaled_throttle
        self.chassis.drive(vx, vz)

    def handle_shooter_inputs(self, joystick: wpilib.Joystick):
        if joystick.getTriggerPressed():
            self.shooter_controller.driver_input(True)
        if joystick.getTriggerReleased():
            self.shooter_controller.driver_input(False)
        if joystick.getRawButtonPressed(5):
            self.shooter_controller.spin_input()

    def handle_hang_inputs(self, joystick: wpilib.Joystick):
        if joystick.getRawButtonPressed(3) and joystick.getRawButtonPressed(4):
            self.hang.raise_hook()
        if self.hang.fire_hook and joystick.getRawButton(4):
            self.hang.winch()

    def testInit(self):
        self.turret.motor.configPeakOutputForward(1.0, 10)
        self.turret.motor.configPeakOutputReverse(-1.0, 10)
        self.test_hall_effect_found = False
        self.testing_hall_effect_width = False

    def testPeriodic(self):

        self.vision.execute()

        self.shooter.execute()

        if self.driver_joystick.getRawButtonPressed(10):
            self.shooter.fire()
            self.indexer.enable_intaking()

        # Slew the turret
        slew_increment = math.radians(5)  # radians
        if self.driver_joystick.getRawButtonPressed(6):
            self.turret.slew(-slew_increment)
            self.turret.execute()
        elif self.driver_joystick.getRawButtonPressed(5):
            self.turret.slew(slew_increment)
            self.turret.execute()
        if self.driver_joystick.getRawButtonPressed(2):
            self.turret.scan()
        if self.turret.current_state == self.turret.SCANNING:
            self.turret.execute()

        # Find the width of a hall effect sensor
        if self.driver_joystick.getRawButtonPressed(12):
            if not self.testing_hall_effect_width:
                # self.turret.motor.set(ctre.ControlMode.PercentOutput, 0.25)
                self.turret.setFindingIndices(False)
                self.testing_hall_effect_width = True
                self.turret.SCAN_CRUISE_VELOCITY = 500
                self.turret.scan()
            else:
                # stop the scanning
                self.turret.slew(0)
                self.turret.setFindingIndices(True)
                self.turret.SCAN_CRUISE_VELOCITY = 1500
                self.testing_hall_effect_width = False

        if self.testing_hall_effect_width:
            if (
                self.turret.centre_index.get() == self.turret.HALL_EFFECT_CLOSED
                and not self.test_hall_effect_found
            ):
                self.start_hall_effect_count = (
                    self.turret.motor.getSelectedSensorPosition()
                )
                self.test_hall_effect_found = True
            if (
                self.turret.centre_index.get() != self.turret.HALL_EFFECT_CLOSED
                and self.test_hall_effect_found
            ):
                self.end_hall_effect_count = (
                    self.turret.motor.getSelectedSensorPosition()
                )
                self.test_hall_effect_found = False
                # self.turret.motor.stopMotor()
                # self.testing_hall_effect_width = False
                print(
                    f"Hall effect detected starting at count: {self.start_hall_effect_count}"
                    + f" and ending at count: {self.end_hall_effect_count}"
                )
            self.turret.execute()

        # Pay out the winch after a match
        if self.driver_joystick.getRawButton(4):
            self.hang.pay_out()
            self.hang.execute()

        if self.driver_joystick.getTrigger():
            self.indexer.enable_intaking()

        if self.driver_joystick.getRawButtonPressed(7):
            self.indexer.shimmy_speed += 0.1
            if self.indexer.shimmy_speed > 1:
                self.indexer.shimmy_speed = 1
            self.indexer.left = True
        if self.driver_joystick.getRawButtonPressed(9):
            self.indexer.shimmy_speed -= 0.1
            if self.indexer.shimmy_speed < 0:
                self.indexer.shimmy_speed = 0
        if self.indexer.intaking:
            self.indexer.intake_motor_speed = (
                self.driver_joystick.getThrottle() + 1
            ) / 2
            self.indexer.execute()


if __name__ == "__main__":
    wpilib.run(MyRobot)
