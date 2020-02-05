from typing import Sequence

from magicbot import feedback
import ctre
import wpilib
from magicbot import feedback, will_reset_to


class Indexer:
    indexer_motors: Sequence[ctre.WPI_TalonSRX]
    piston_switch: wpilib.DigitalInput

    intake_arm_piston: wpilib.Solenoid
    intake_main_motor: ctre.WPI_TalonSRX
    intake_left_motor: wpilib.Spark  # Looking from behind the robot
    intake_right_motor: wpilib.Spark  # Looking from behind the robot
    SHIMMY_TICKS = int(50 * 0.5)

    def setup(self):
        for motor in self.indexer_motors:
            motor.setInverted(True)
            motor.setNeutralMode(ctre.NeutralMode.Brake)
            motor.configForwardLimitSwitchSource(
                ctre.LimitSwitchSource.FeedbackConnector,
                ctre.LimitSwitchNormal.NormallyOpen,
            )

        self.intake_main_motor.setInverted(True)
        self.intake_left_motor.setInverted(False)
        self.intake_right_motor.setInverted(False)

        self.indexer_motors[-1].setInverted(False)

        self.indexer_speed = 0.6
        self.injector_speed = 0.7

        # We have a delay because the distance between the second last
        # stage of the indexer and the injector is longer than others
        # and the injector motors are slower
        self.transfer_to_injector = False
        self.intake_lowered = False
        self.intaking = False
        self.left_shimmy = True

        self.shimmy_count = 0
        self.intake_motor_speed = 1.0
        self.shimmy_speed = 1.0
        self.intaking = False

    def on_enable(self) -> None:
        self.shimmy_count = 0
        self.intaking = True

    def execute(self) -> None:
        if self.intaking:
            injector = self.indexer_motors[-1]
            feeder = self.indexer_motors[-2]
            if injector.isFwdLimitSwitchClosed():
                self.transfer_to_injector = False
            elif feeder.isFwdLimitSwitchClosed():
                # Transferring
                self.transfer_to_injector = True

            # Turn on all motors and let the limit switches stop it
            for motor in self.indexer_motors[:-2]:
                motor.set(self.indexer_speed)
            if self.is_piston_retracted():
                feeder.set(self.indexer_speed)
                injector.set(self.injector_speed)
            else:
                feeder.stopMotor()
                injector.stopMotor()

            # Override any limit switches where the next cell is vacant
            for first, second in zip(self.indexer_motors, self.indexer_motors[1:]):
                at_limit = second.isFwdLimitSwitchClosed()
                if second == feeder and self.transfer_to_injector:
                    at_limit = True  # Pretend the ball is still in the feeder
                if not at_limit:
                    first.overrideLimitSwitchesEnable(False)
                else:
                    first.overrideLimitSwitchesEnable(True)

            self.intake_main_motor.set(self.intake_motor_speed)
            if self.left_shimmy:
                self.intake_left_motor.set(self.shimmy_speed)
                self.intake_right_motor.set(0)
                self.shimmy_count += 1
                if self.shimmy_count > self.SHIMMY_TICKS:
                    self.left_shimmy = False
                    self.shimmy_count = 0
            else:
                self.intake_left_motor.set(0)
                self.intake_right_motor.set(self.shimmy_speed)
                self.shimmy_count += 1
                if self.shimmy_count > self.SHIMMY_TICKS:
                    self.left_shimmy = True
                    self.shimmy_count = 0
        else:
            # Move balls through if we have them, but don't take in more
            ball_in_previous = False
            for motor in self.indexer_motors:
                if not motor.isFwdLimitSwitchClosed():
                    # We don't have a ball in this cell
                    # Test this first so the previous ball flag works
                    if not ball_in_previous:
                        motor.stopMotor()
                else:
                    ball_in_previous = True
                motor.stopMotor()

        if self.intake_lowered:
            self.intake_arm_piston.set(True)
        else:
            self.intake_arm_piston.set(False)

    def enable_intaking(self) -> None:
        self.intaking = True

    def disable_intaking(self) -> None:
        self.intaking = False

    def raise_intake(self) -> None:
        self.intake_lowered = False

    def lower_intake(self) -> None:
        self.intake_lowered = True

    @feedback
    def is_intake_lowered(self) -> bool:
        return self.intake_lowered

    @feedback
    def balls_loaded(self) -> int:
        balls = sum(motor.isFwdLimitSwitchClosed() for motor in self.indexer_motors)
        return balls

    def is_piston_retracted(self) -> bool:
        return not self.piston_switch.get()

    @feedback
    def is_ready(self) -> bool:
        return self.indexer_motors[-1].isFwdLimitSwitchClosed()
