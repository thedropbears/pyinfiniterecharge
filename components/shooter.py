import wpilib
from wpimath.controller import SimpleMotorFeedforward
import ctre
from numpy import interp
from magicbot import feedback, tunable

from components.indexer import Indexer


class Shooter:
    outer_motor: ctre.WPI_TalonFX
    centre_motor: ctre.WPI_TalonFX  # centre of arc
    loading_piston: wpilib.DoubleSolenoid
    piston_switch: wpilib.DigitalInput

    indexer: Indexer

    ranges = (2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13)  # TODO add more data points
    centre_lookup = (28, 30, 42, 48, 55, 64, 68, 70, 75, 80, 87, 95)
    outer_lookup = (65, 53, 40, 36, 28, 22, 20, 20, 19, 18, 18, 17)

    outer_target = tunable(0)
    centre_target = tunable(0)

    COUNTS_PER_REV = 2048
    RPS_TO_CTRE_UNITS = COUNTS_PER_REV / 10  # counts per 100ms
    CTRE_UNITS_TO_RPS = 1 / RPS_TO_CTRE_UNITS

    def __init__(self):
        self.inject = False
        self.in_range = False
        self.velocity_tolerance = 0.05  # of setpoint
        self.disabled = False

    def setup(self) -> None:
        self.outer_motor.setInverted(True)
        self.centre_motor.setInverted(False)

        for motor in self.outer_motor, self.centre_motor:
            motor.configVelocityMeasurementPeriod(ctre.VelocityMeasPeriod.Period_1Ms)
            motor.configVelocityMeasurementWindow(16)
            motor.setNeutralMode(ctre.NeutralMode.Coast)

        self.outer_motor.config_kP(0, 0.232)
        self.outer_motor.config_kI(0, 0)
        self.outer_motor.config_kD(0, 0)
        self.outer_motor.config_kF(0, 0)
        self.outer_ff_calculator = SimpleMotorFeedforward(kS=0.39069, kV=0.10715)
        self.centre_motor.config_kP(0, 0.286)
        self.centre_motor.config_kI(0, 0)
        self.centre_motor.config_kD(0, 0)
        self.centre_motor.config_kF(0, 0)
        self.centre_ff_calculator = SimpleMotorFeedforward(kS=0.41315, kV=0.1069)

    def on_disable(self) -> None:
        self.inject = False
        self.loading_piston.set(wpilib.DoubleSolenoid.Value.kOff)

    def execute(self) -> None:
        if self.disabled:
            self.centre_motor.stopMotor()
            self.outer_motor.stopMotor()
            return

        voltage = wpilib.RobotController.getInputVoltage()
        centre_feed_forward = self.centre_ff_calculator.calculate(self.centre_target)
        outer_feed_forward = self.outer_ff_calculator.calculate(self.outer_target)

        self.centre_motor.set(
            ctre.ControlMode.Velocity,
            self.centre_target * self.RPS_TO_CTRE_UNITS,
            ctre.DemandType.ArbitraryFeedForward,
            centre_feed_forward / voltage,
        )
        self.outer_motor.set(
            ctre.ControlMode.Velocity,
            self.outer_target * self.RPS_TO_CTRE_UNITS,
            ctre.DemandType.ArbitraryFeedForward,
            outer_feed_forward / voltage,
        )
        # self.centre_motor.setVoltage(centre_feed_forward)
        # self.outer_motor.setVoltage(outer_feed_forward)

        if self.inject:
            self.loading_piston.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.loading_piston.set(wpilib.DoubleSolenoid.Value.kReverse)
        if self.is_ball_cleared() and self.inject:
            self.inject = False

    def set_range(self, dist: float) -> None:
        """
        Set the target range for the shooter, this will be converted into target speeds for the flywheels
        dist: planar distance from the power port
        """
        if self.ranges[0] <= dist <= self.ranges[-1]:
            self.in_range = True
        else:
            # clamp the range between our minimum and maximum
            dist = min(self.ranges[-1], max(dist, self.ranges[0]))
            self.in_range = False
        self.centre_target = interp(dist, self.ranges, self.centre_lookup)
        self.outer_target = interp(dist, self.ranges, self.outer_lookup)

    @feedback
    def is_at_speed(self) -> bool:
        """
        Returns true if the shooter is spinning at the set speed.

        Considers the rotation rates of the flywheels compared with their setpoints
        """
        if self.centre_target <= 0 or self.outer_target <= 0:
            return False
        return self.is_centre_at_speed() and self.is_outer_at_speed()

    @feedback
    def is_centre_at_speed(self) -> bool:
        return (
            abs(self.centre_target - self.get_centre_velocity())
            <= self.centre_target * self.velocity_tolerance
        )

    @feedback
    def is_outer_at_speed(self) -> bool:
        return (
            abs(self.outer_target - self.get_outer_velocity())
            <= self.outer_target * self.velocity_tolerance
        )

    @feedback
    def get_centre_velocity(self):
        """Returns velocity in rps"""
        return self.centre_motor.getSelectedSensorVelocity() * self.CTRE_UNITS_TO_RPS

    @feedback
    def get_outer_velocity(self):
        """Returns velocity in rps"""
        return self.outer_motor.getSelectedSensorVelocity() * self.CTRE_UNITS_TO_RPS

    @feedback
    def is_firing(self) -> bool:
        """
        Returns true if the shooter is attempting a shot.

        based off of the pistons current state
        """
        return self.piston_switch.get()

    @feedback
    def is_in_range(self) -> bool:
        """
        Returns true if the current target of the shooter is within range
        Returns false if the range has been clamped
        """
        return self.in_range

    @feedback
    def is_ready(self) -> bool:
        """
        Returns true if the shooter is ready to take a shot.

        Checks the speed, range and whether the piston is moving
        """
        # print(f"in range {self.is_in_range()} at speed {self.is_at_speed()} is firing {self.is_firing()}")
        return self.is_in_range() and self.is_at_speed() and not self.is_firing()

    def fire(self) -> None:
        """
        Inject a ball into the shooter
        """
        self.inject = True

    def is_ball_cleared(self) -> bool:
        return not self.indexer.injector_has_ball()

    def toggle(self) -> None:
        """Toggle the shooter on and off"""
        self.disabled = not self.disabled
