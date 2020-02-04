from enum import Enum

import wpilib
import ctre
import math


class Turret:
    # TODO - There should be 4 indexes total: left, right, front and rear hall-effect
    # sensors. Right now there is only one hall-effect sensor at the front.
    # left_index: wpilib.DigitalInput
    # right_index: wpilib.DigitalInput
    centre_index: wpilib.DigitalInput
    HALL_EFFECT_CLOSED = False

    motor: ctre.WPI_TalonSRX

    # Possible states
    SLEWING = 0
    SCANNING = 1

    # Constants for Talon on the turret
    COUNTS_PER_MOTOR_REV = 4096
    GEAR_REDUCTION = 160 / 18
    COUNTS_PER_TURRET_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION
    COUNTS_PER_TURRET_RADIAN = COUNTS_PER_TURRET_REV / math.tau

    # PID values
    pidF = 0
    pidP = 0.2
    pidI = 0
    pidD = 0

    # Slew to within +- half a degree of the target azimuth. This is about
    # 50 encoder steps.
    CLOSED_LOOP_ERROR = int(math.radians(0.5) * COUNTS_PER_TURRET_RADIAN)
    # The number of cycles that we must be within the error to decide we're done.
    TICKS_TO_SETTLE = 10

    def on_enable(self) -> None:
        self.scan()

    def setup(self) -> None:
        self.motor.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10
        )
        self.motor.config_kF(0, self.pidF, 10)
        self.motor.config_kP(0, self.pidP, 10)
        self.motor.config_kI(0, self.pidI, 10)
        self.motor.config_kD(0, self.pidD, 10)
        self.motor.configAllowableClosedloopError(0, self.CLOSED_LOOP_ERROR, 10)
        self.scan_increment = math.radians(10.0)
        self.index_found = False
        self.current_state = self.SLEWING

    # Slew to the given absolute angle (in radians). An angle of 0 corresponds
    # to the centre index point.
    def slew_to_azimuth(self, angle: float) -> None:
        if self.index_found:
            self.current_state = self.SLEWING
            self.motor._slew_to_counts(angle * self.COUNTS_PER_TURRET_RADIAN)
        else:
            self.logger.warning("slew_to_azimuth() called before index found")

    # Slew the given angle (in radians) from the current position
    def slew(self, angle: float) -> None:
        self.current_state = self.SLEWING
        current_pos = self.motor.getClosedLoopTarget()
        self._slew_to_counts(current_pos + angle * self.COUNTS_PER_TURRET_RADIAN)

    def _slew_to_counts(self, counts: int) -> None:
        # TODO: Check for values outside allowed range
        self.motor.set(ctre.ControlMode.Position, counts)

    def scan(self, azimuth=0.0) -> None:
        """
        Slew the turret back and forth looking for a target.
        """
        # If we haven't hit an index yet, we just have to scan
        # about the current position.
        # Otherwise scan about the heading we've been given.
        # The target must be downfield from us, so scan up to
        # 90 degrees either side of the given heading

        # First reset scan size
        self.current_scan_delta = self.scan_increment
        if self.index_found:
            # set the first pass
            self._slew_to_counts(
                (azimuth + self.scan_increment) * self.COUNTS_PER_TURRET_RADIAN
            )
        else:
            current_count = self.motor.getSelectedSensorPosition()
            self._slew_to_counts(
                current_count + (self.scan_increment * self.COUNTS_PER_TURRET_RADIAN)
            )

    def is_ready(self) -> bool:
        return (
            self.current_state != self.SCANNING
            and abs(self.motor.getClosedLoopError) < self.ACCEPTABLE_ERROR_COUNTS
        )

    def has_index(self) -> bool:
        return self.index_found

    def execute(self) -> None:
        self._check_for_index()
        if self.current_state == self.SCANNING:
            self._do_scanning()

    def _check_for_index(self) -> None:
        # Check if we're at a known position
        # If so, update the encoder position on the motor controller
        # and change the current setpoint with the applied delta.
        if self.centre_index.get() == self.HALL_EFFECT_CLOSED:
            _reset_encoder(0)
        # TODO: Repeat for other index marks

    def _reset_encoder(self, counts) -> None:
        current_count = self.motor.getSelectedSensorPosition()
        current_target = self.motor.getClosedLoopTarget()
        delta = current_target - current_count
        self.motor.setSelectedSensorPosition(counts)
        # Reset any current target using the new absolute azimuth
        self._slew_to_counts(counts + delta)
        self.has_index = True

    def _do_scanning(self) -> None:
        # Check if we've finished a scan pass
        # If so, reverse the direction and increase pass size if necessary
        if abs(self.motor.getClosedLoopError()) < self.ACCEPTABLE_ERROR_COUNTS:
            next_target = self.motor.getClosedLoopTarget() - self.current_scan_delta
            # next_target points back at the centre of the scan again
            if 0 < self.current_scan_delta < math.pi() / 2:
                self.current_scan_delta = self.current_scan_delta + self.scan_increment
            if -math.pi() / 2 < self.current_scan_delta < 0:
                self.current_scan_delta = self.current_scan_delta - self.scan_increment
            self.current_scan_delta = -self.current_scan_delta
            next_target = next_target + self.current_scan_delta
            self._slew_to_counts(next_target * self.COUNTS_PER_TURRET_RADIAN)
