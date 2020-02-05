from enum import Enum

import wpilib
import ctre
import math


class Index(Enum):
    NOT_FOUND = 0
    CENTRE = 1
    RIGHT = 2
    LEFT = 3
    BACK = 4


class Turret:
    # TODO - There should be 4 indexes total: left, right, front and rear hall-effect
    # sensors. Right now there is only one hall-effect sensor at the front.
    # left_index: wpilib.DigitalInput
    # right_index: wpilib.DigitalInput
    centre_index: wpilib.DigitalInput
    motor: ctre.WPI_TalonSRX

    # Possible states
    SLEWING = 0
    SCANNING = 1

    # Constants for Talon on the turret
    COUNTS_PER_MOTOR_REV = 4096
    GEAR_REDUCTION = 160 / 18
    COUNTS_PER_TURRET_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION
    COUNTS_PER_TURRET_RADIAN = COUNTS_PER_TURRET_REV / math.tau

    INDEX_POSITIONS = {
        Index.CENTRE: 0,
        Index.LEFT: int(math.pi / 2 * COUNTS_PER_TURRET_RADIAN),
        Index.RIGHT: int(-math.pi / 2 * COUNTS_PER_TURRET_RADIAN),
        Index.BACK: int(math.pi * COUNTS_PER_TURRET_RADIAN),
    }
    HALL_EFFECT_CLOSED = False
    HALL_EFFECT_HALF_WIDTH_COUNTS = 100  # TODO: Check this on the robot

    # TLimit to prevent turret from rotating too far.
    # This assumes that the centre index is at a count of 0.
    # Temporarily imit the travel to 45 degrees away from 0 while debugging.
    MAX_TURRET_COUNT = math.radians(45) * COUNTS_PER_TURRET_RADIAN

    # PID values
    pidF = 0
    pidP = 0.2
    pidI = 0
    pidD = 0

    # Slew to within +- half a degree of the target azimuth. This is about
    # 50 encoder steps.
    ACCEPTABLE_ERROR_COUNTS = int(math.radians(0.5) * COUNTS_PER_TURRET_RADIAN)
    ACCEPTABLE_ERROR_SPEED = int(
        math.radians(0.5) * COUNTS_PER_TURRET_RADIAN
    )  # counts per 100ms

    def on_enable(self) -> None:
        if self.index_found:
            # Don't throw away previously found indices
            self.motor.set(
                ctre.ControlMode.Position, self.motor.getSelectedSensorPosition()
            )
        else:
            self.motor.setSelectedSensorPosition(0)
            self.motor.set(ctre.ControlMode.Position, 0)

    def setup(self) -> None:
        self.motor.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10
        )
        self.motor.config_kF(0, self.pidF, 10)
        self.motor.config_kP(0, self.pidP, 10)
        self.motor.config_kI(0, self.pidI, 10)
        self.motor.config_kD(0, self.pidD, 10)
        self.motor.configAllowableClosedloopError(0, self.ACCEPTABLE_ERROR_COUNTS, 10)
        self.scan_increment = math.radians(10.0)
        self.index_found = False
        self.previous_index = Index.NOT_FOUND
        self.current_state = self.SLEWING

    # Slew to the given absolute angle (in radians). An angle of 0 corresponds
    # to the centre index point.
    def slew_to_azimuth(self, angle: float) -> None:
        if self.index_found:
            self.current_state = self.SLEWING
            self.motor._slew_to_counts(angle * self.COUNTS_PER_TURRET_RADIAN)
        else:
            # self.logger.warning(
            #    "slew_to_azimuth() called before index found"
            # )  # pylint: disable=no-member
            print("slew_to_azimuth() called before index found")

    # Slew the given angle (in radians) from the current position
    def slew(self, angle: float) -> None:
        self.current_state = self.SLEWING
        current_pos = self.motor.getClosedLoopTarget()
        self._slew_to_counts(current_pos + angle * self.COUNTS_PER_TURRET_RADIAN)

    def _slew_to_counts(self, counts: int) -> None:
        if -self.MAX_TURRET_COUNT < counts < self.MAX_TURRET_COUNT:
            self.motor.set(ctre.ControlMode.Position, counts)
        else:
            print("attempt to slew beyond limit: %d", counts)
            # self.logger.warning(
            #    "attempt to slew beyond limit: " + counts
            # )  # pylint: disable=no-member

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
        self.current_state = self.SCANNING

    def is_ready(self) -> bool:
        return (
            self.current_state != self.SCANNING
            and abs(self.motor.getClosedLoopError()) < self.ACCEPTABLE_ERROR_COUNTS
            and abs(self.motor.getSelectedSensorVelocity())
            < self.ACCEPTABLE_ERROR_SPEED
        )

    def has_index(self) -> bool:
        return self.index_found

    def execute(self) -> None:
        self._handle_indices()
        if self.current_state == self.SCANNING:
            self._do_scanning()

    def _handle_indices(self) -> None:
        # Check if we're at a known position
        # If so, update the encoder position on the motor controller
        # and change the current setpoint with the applied delta.
        index = self._get_current_index()
        if index != Index.NOT_FOUND and self.previous_index == Index.NOT_FOUND:
            # Last tick we didn't have an index triggered, and now we do
            count = self._index_to_counts(index)
            self._reset_encoder(count)
        self.previous_index = index

    def _get_current_index(self) -> Index:
        if self.centre_index.get() == self.HALL_EFFECT_CLOSED:
            return Index.CENTRE
        # TODO: Repeat for other index marks
        return Index.NOT_FOUND

    def _index_to_counts(self, index: Index) -> int:
        # We need to account for the width of the sensor.
        # This means we use the direction of travel to work out
        # which side we are approaching from
        middle = self.INDEX_POSITIONS[index]
        direction = 1 if self.motor.getSelectedSensorVelocity() > 0 else -1
        count = middle + direction * self.HALL_EFFECT_HALF_WIDTH_COUNTS
        return count

    def _reset_encoder(self, counts) -> None:
        current_count = self.motor.getSelectedSensorPosition()
        current_target = self.motor.getClosedLoopTarget()
        delta = current_target - current_count
        self.motor.setSelectedSensorPosition(counts)
        # Reset any current target using the new absolute azimuth
        self._slew_to_counts(counts + delta)
        self.index_found = True

    def _do_scanning(self) -> None:
        # Check if we've finished a scan pass
        # If so, reverse the direction and increase pass size if necessary
        if abs(self.motor.getClosedLoopError()) < self.ACCEPTABLE_ERROR_COUNTS:
            next_target = self.motor.getClosedLoopTarget() - self.current_scan_delta
            # next_target points back at the centre of the scan again
            if 0 < self.current_scan_delta < math.pi / 2:
                self.current_scan_delta = self.current_scan_delta + self.scan_increment
            if -math.pi / 2 < self.current_scan_delta < 0:
                self.current_scan_delta = self.current_scan_delta - self.scan_increment
            self.current_scan_delta = -self.current_scan_delta
            next_target = next_target + self.current_scan_delta
            self._slew_to_counts(next_target * self.COUNTS_PER_TURRET_RADIAN)
