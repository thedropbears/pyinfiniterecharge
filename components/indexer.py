from magicbot import feedback
import ctre
import wpilib


class Indexer:
    indexer_motors: list

    def setup(self):
        for motor in self.indexer_motors:
            motor.setInverted(False)
            motor.configForwardLimitSwitchSource(ctre.LimitSwitchSource.FeedbackConnector, ctre.LimitSwitchNormal.NormallyOpen)

        self.indexer_speed = 0.2

    def on_enable(self) -> None:
        self.intaking = True

    def execute(self) -> None:
        if self.intaking:
            # Turn on all motors and let the limit switches stop it
            for motor in self.indexer_motors:
                motor.set(self.indexer_speed)

            # Override any limit switches where the next cell is vacant
            for first, second in zip(self.indexer_motors, self.indexer_motors[1:]):
                first.overrideLimitSwitchesEnable(True)
                at_limit = second.isFwdLimitSwitchClosed()
                if not at_limit:
                    first.overrideLimitSwitchesEnable(False)
        else:
            # Move balls through if we have them, but don't take in more
            self.injector_master_motor.stopMotor()
            ball_in_previous = False
            for motor in self.indexer_motors:
                if not motor.isFwdLimitSwitchClosed():
                    # We don't have a ball in this cell
                    # Test this first so the previous ball flag works
                    if not ball_in_previous:
                        motor.stopMotor()
                else:
                    ball_in_previous = True

    def enable_intaking(self) -> None:
        self.intaking = True

    def disable_intaking(self) -> None:
        self.intaking = False

    @feedback
    def balls_loaded(self) -> int:
        balls = sum(motor.isFwdLimitSwitchClosed() for motor in self.indexer_motors)
        return balls

    @feedback
    def is_ready(self) -> bool:
        return  self.indexer_motors[-1].isFwdLimitSwitchClosed()
