#!/usr/bin/env python3

import math

import ctre
import magicbot
import navx
import rev
import wpilib

from wpilib.interfaces import GenericHID
from wpimath import geometry

from components.chassis import Chassis
from components.hang import Hang
from components.indexer import Indexer
from components.range_finder import RangeFinder
from components.shooter import Shooter
from components.target_estimator import TargetEstimator
from components.turret import Turret
from components.vision import Vision
from components.led_screen import LEDScreen
from controllers.shooter import ShooterController
from controllers.kraken import KrakenController
from utilities import git
from utilities.scalers import rescale_js, scale_value

GIT_COMMIT = git.describe()


class MyRobot(magicbot.MagicRobot):
    # List sensors which should collect data before controllers here.
    range_finder: RangeFinder
    vision: Vision

    # List controllers (which require components) here.
    target_estimator: TargetEstimator
    shooter_controller: ShooterController
    kraken_controller: KrakenController

    # List components (which represent physical subsystems) here.
    chassis: Chassis
    hang: Hang
    indexer: Indexer
    shooter: Shooter
    turret: Turret
    led_screen: LEDScreen

    MANUAL_SLEW_SPEED = 5 / 50  # in radians per tick

    def createObjects(self):
        """Robot initialization function"""
        self.logger.info("pyinfiniterecharge %s", GIT_COMMIT)

        self.chassis_left_front = rev.CANSparkMax(5, rev.MotorType.kBrushless)
        self.chassis_left_rear = rev.CANSparkMax(6, rev.MotorType.kBrushless)
        self.chassis_right_front = rev.CANSparkMax(7, rev.MotorType.kBrushless)
        self.chassis_right_rear = rev.CANSparkMax(4, rev.MotorType.kBrushless)
        self.imu = navx.AHRS.create_spi(update_rate_hz=50)

        self.hang_winch_motor = ctre.WPI_TalonFX(30)
        self.hang_kracken_hook_latch = wpilib.Solenoid(0)

        self.intake_main_motor = ctre.WPI_TalonSRX(18)
        self.indexer_motors = [
            ctre.WPI_TalonSRX(11),
            ctre.WPI_TalonSRX(12),
            ctre.WPI_TalonSRX(13),
        ]
        self.injector_motor = ctre.WPI_TalonSRX(14)
        self.piston_switch = wpilib.DigitalInput(4)  # checks if injector retracted
        self.intake_arm_piston = wpilib.Solenoid(1)
        self.intake_left_motor = rev.CANSparkMax(8, rev.MotorType.kBrushless)
        self.intake_right_motor = rev.CANSparkMax(9, rev.MotorType.kBrushless)

        self.led_screen_led = wpilib.AddressableLED(0)
        self.loading_piston = wpilib.DoubleSolenoid(2, 3)
        self.shooter_centre_motor = ctre.WPI_TalonFX(2)
        self.shooter_outer_motor = ctre.WPI_TalonFX(3)

        self.range_counter = wpilib.Counter(6)

        self.turret_centre_index = wpilib.DigitalInput(0)
        self.turret_right_index = wpilib.DigitalInput(1)
        self.turret_left_index = wpilib.DigitalInput(2)
        self.turret_motor = ctre.WPI_TalonSRX(10)

        # operator interface
        self.driver_joystick = wpilib.Joystick(0)
        self.codriver_gamepad = wpilib.XboxController(1)

        self.MEMORY_CONSTANT = int(0.1 / self.control_loop_wait_time)
        # how long before data times out

        self.has_zeroed = False

    def autonomousInit(self) -> None:
        """Initialise things for all autonomous modes."""
        self.chassis.enable_closed_loop()
        self.chassis.enable_brake_mode()
        self.indexer.shimmying = False
        self.indexer.auto_retract = False

    def disabledPeriodic(self) -> None:
        self.vision.execute()

    def teleopInit(self) -> None:
        """Initialise things for driver control."""
        if not self.has_zeroed:
            self.chassis.reset_odometry(
                geometry.Pose2d(-3, 0, geometry.Rotation2d(math.pi))
            )
            self.has_zeroed = True
        self.chassis.disable_closed_loop()
        self.chassis.disable_brake_mode()
        self.indexer.shimmying = True
        self.indexer.auto_retract = True

    def teleopPeriodic(self):
        """Executed every cycle"""
        self.handle_intake_inputs(self.driver_joystick, self.codriver_gamepad)
        self.handle_chassis_inputs(self.driver_joystick, self.codriver_gamepad)
        self.handle_shooter_inputs(self.driver_joystick, self.codriver_gamepad)
        self.handle_hang_inputs(self.driver_joystick, self.codriver_gamepad)

        self.shooter_controller.engage()

    def handle_intake_inputs(
        self, joystick: wpilib.Joystick, gamepad: wpilib.XboxController
    ) -> None:
        if joystick.getRawButtonPressed(2):
            if self.indexer.intaking:
                self.indexer.disable_intaking()
                self.indexer.raise_intake()
            else:
                self.indexer.enable_intaking()
                self.indexer.lower_intake()

        if gamepad.getAButton():
            # Dump all balls out the intake to try to clear jam, etc
            self.indexer.clearing = True
        else:
            # Normal operation
            self.indexer.clearing = False

        if gamepad.getBButton():
            # Reverse only intake to clear an intake jam without losing other balls
            self.indexer.intake_clearing = True
        else:
            self.indexer.intake_clearing = False

    def handle_chassis_inputs(
        self, joystick: wpilib.Joystick, gamepad: wpilib.XboxController
    ) -> None:
        throttle = scale_value(joystick.getThrottle(), 1, -1, 0.1, 1)
        vx = 3 * throttle * rescale_js(-joystick.getY(), 0.1)
        vz = 3 * throttle * rescale_js(-joystick.getTwist(), 0.1)
        self.chassis.drive(vx, vz)
        if joystick.getRawButtonPressed(3):
            self.chassis.reset_odometry(
                Pose2d(-3, 0, Rotation2d(math.pi))
            )  # the starting position on the field
            self.target_estimator.reset()

    def handle_shooter_inputs(
        self, joystick: wpilib.Joystick, gamepad: wpilib.XboxController
    ) -> None:
        if joystick.getTrigger():
            self.shooter_controller.fire_input()
        if gamepad.getBackButton() and gamepad.getRawButtonPressed(5):
            # Disable turret in case of catastrophic malfunction
            # Make this toggle to allow re-enabling turret in case it was accidentally disabled
            self.shooter.toggle()
            self.turret.toggle()

        # Hold to stay in manual aiming mode
        if gamepad.getXButton():
            self.shooter_controller.is_manual_aiming = True
            slew_amount = (
                rescale_js(gamepad.getX(GenericHID.Hand.kLeftHand), 0.1)
                * -self.MANUAL_SLEW_SPEED
            )
            self.shooter_controller.manual_slew(slew_amount)

    def handle_hang_inputs(
        self, joystick: wpilib.Joystick, gamepad: wpilib.XboxController
    ) -> None:
        if gamepad.getStartButton() and gamepad.getBumper(GenericHID.Hand.kRightHand):
            self.kraken_controller.engage()
        if self.hang.fire_hook and (
            gamepad.getTriggerAxis(GenericHID.Hand.kLeftHand) > 0.9
            or gamepad.getTriggerAxis(GenericHID.Hand.kRightHand) > 0.9
        ):
            self.hang.winch()

    def testInit(self):
        self.turret.index_found = False
        self.turret.on_enable()
        self.track_target = False
        self.run_indexer = False
        self.chassis.reset_odometry(
            geometry.Pose2d(1, -1, geometry.Rotation2d(math.pi))
        )

    track_target = magicbot.tunable(False)

    def testPeriodic(self):

        self.vision.execute()

        if self.track_target:
            self.target_estimator.execute()
            self.shooter_controller.engage()
            self.shooter_controller.execute()
            self.turret.execute()

        self.shooter.execute()

        if self.run_indexer:
            self.indexer.execute()

        if self.driver_joystick.getRawButtonPressed(2):
            self.run_indexer = True
            if self.indexer.intaking:
                self.indexer.disable_intaking()
                self.indexer.raise_intake()
            else:
                self.indexer.enable_intaking()
                self.indexer.lower_intake()

        if self.driver_joystick.getRawButtonPressed(3):
            self.track_target = not self.track_target

        # Pay out the winch after a match
        if self.driver_joystick.getRawButton(4):
            self.hang.pay_out()
            self.hang.execute()

        # Slew the turret
        slew_increment = math.radians(5)  # radians
        if self.driver_joystick.getRawButtonPressed(5):
            self.turret.slew(slew_increment)
            self.turret.execute()
        elif self.driver_joystick.getRawButtonPressed(6):
            self.turret.slew(-slew_increment)
            self.turret.execute()

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

        if self.driver_joystick.getTriggerPressed():
            self.shooter.fire()


if __name__ == "__main__":
    wpilib.run(MyRobot)
