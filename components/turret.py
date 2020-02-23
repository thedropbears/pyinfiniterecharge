import math
import time
from collections import deque
from enum import Enum
from typing import Optional

import wpilib
import ctre


class Index(Enum):
    # These are relative to the turret, which faces backwards on the robot.
    NOT_FOUND = 0
    CENTRE = 1
    RIGHT = 2
    LEFT = 3


class Turret:
    centre_index: wpilib.DigitalInput
    right_index: wpilib.DigitalInput
    left_index: wpilib.DigitalInput
    motor: ctre.WPI_TalonSRX

    MEMORY_CONSTANT: int
    control_loop_wait_time: float

    # Possible states
    SLEWING = 0
    SCANNING = 1

    # Constants for Talon on the turret
    COUNTS_PER_MOTOR_REV = 4096
    GEAR_REDUCTION = 160 / 18
    COUNTS_PER_TURRET_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION
    COUNTS_PER_TURRET_RADIAN = int(COUNTS_PER_TURRET_REV / math.tau)

    INDEX_POSITIONS = {
        # Names are relative to the turret, but the count is kept at 0
        # when the turret is facing backwards on the robot.
        Index.CENTRE: 0,
        Index.LEFT: int(math.pi / 2 * COUNTS_PER_TURRET_RADIAN),
        Index.RIGHT: int(-math.pi / 2 * COUNTS_PER_TURRET_RADIAN),
    }
    HALL_EFFECT_CLOSED = False
    HALL_EFFECT_HALF_WIDTH_COUNTS = 100  # TODO: Check this on the robot

    # Limit to prevent turret from rotating too far. Note that this allows for
    # a total coverage > 360 degrees.
    MAX_TURRET_COUNT = int(math.radians(190) * COUNTS_PER_TURRET_RADIAN)

    # PID values
    # Open loop tests give a turret speed of ~930counts/100ms at 25% throttle
    # ~ 2500 at 50% throttle
    # kF = (1023 * 50%) / 2500 ~= 0.2
    # A commanded move with a cruise of 2500 reaches about 1500
    # Error of 1000. Add 10% more throttle at this point
    # kP = (1023 * 10%) / 1000 ~= 0.1
    # Double until oscillation occurs
    # Set kD to 10*kP
    pidF = 0.2
    pidP = 0.4
    pidI = 0.005
    pidIZone = 300
    pidD = 4.0
    SLEW_CRUISE_VELOCITY = 3500
    SCAN_CRUISE_VELOCITY = 1500
    CRUISE_ACCELERATION = int(SLEW_CRUISE_VELOCITY / 0.15)

    # Slew to within +- half a degree of the target azimuth. This is about
    # 50 encoder steps.
    ACCEPTABLE_ERROR_COUNTS = int(math.radians(0.5) * COUNTS_PER_TURRET_RADIAN)
    ACCEPTABLE_ERROR_SPEED = int(
        math.radians(0.5) * COUNTS_PER_TURRET_RADIAN
    )  # counts per 100ms

    PI_OVER_4_IN_COUNTS = int(math.pi / 4 * COUNTS_PER_TURRET_RADIAN)

    def on_enable(self) -> None:
        self.motor.configPeakOutputForward(1.0, 10)
        self.motor.configPeakOutputReverse(-1.0, 10)
        if self.index_found:
            # Don't throw away previously found indices
            self.motor.set(
                ctre.ControlMode.MotionMagic, self.motor.getSelectedSensorPosition()
            )
        else:
            self.motor.setSelectedSensorPosition(0)
            self.motor.set(ctre.ControlMode.MotionMagic, 0)
            # Handle the case where we start out on an index.
            self._handle_indices()

    def setup(self) -> None:
        self.motor.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10
        )
        # Positive motion is counterclockwise from above.
        self.motor.setInverted(ctre.InvertType.InvertMotorOutput)
        # set the peak and nominal outputs
        self.motor.configNominalOutputForward(0, 10)
        self.motor.configNominalOutputReverse(0, 10)
        self.motor.configPeakOutputForward(1.0, 10)
        self.motor.configPeakOutputReverse(-1.0, 10)
        # Set relevant frame periods to be at least as fast as periodic rate
        self.motor.setStatusFramePeriod(
            ctre.StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10
        )
        self.motor.setStatusFramePeriod(
            ctre.StatusFrameEnhanced.Status_10_MotionMagic, 10, 10
        )
        self.motor.config_kF(0, self.pidF, 10)
        self.motor.config_kP(0, self.pidP, 10)
        self.motor.config_IntegralZone(0, self.pidIZone, 10)
        self.motor.config_kI(0, self.pidI, 10)
        self.motor.config_kD(0, self.pidD, 10)

        self.motor.configMotionCruiseVelocity(self.SLEW_CRUISE_VELOCITY, 10)
        self.motor.configMotionAcceleration(self.CRUISE_ACCELERATION, 10)
        # self.motor.configAllowableClosedloopError(0, self.ACCEPTABLE_ERROR_COUNTS, 10)
        self.current_target_counts = 0
        self.scan_increment = int(math.radians(10.0) * self.COUNTS_PER_TURRET_RADIAN)
        self.index_found = False
        self.previous_index = Index.NOT_FOUND
        self.current_state = self.SLEWING

        self.azimuth_history = deque(maxlen=self.MEMORY_CONSTANT)

    # Slew to the given absolute angle in radians in the robot coordinate system.
    # We assume that the angle is between -pi and pi
    # If no index has been seen yet, we have no reference for the angle, so we
    # ignore the command.
    #
    # TODO: perhaps we should return an error so the calling code can know?
    def slew_to_azimuth(self, angle: float) -> None:
        angle -= math.pi  # This converts to turret coordinate system
        # Now we normalise to +- pi
        while angle < -math.pi:
            angle += math.tau
        while angle > math.pi:
            angle -= math.tau
        if self.index_found:
            self.current_state = self.SLEWING
            self.motor._slew_to_counts(int(angle * self.COUNTS_PER_TURRET_RADIAN))
        else:
            print("slew_to_azimuth() called before index found")

    # Slew the given angle (in radians) from the current position
    def slew(self, angle: float) -> None:
        self.current_state = self.SLEWING
        current_pos = self.motor.getSelectedSensorPosition()
        target = current_pos + int(angle * self.COUNTS_PER_TURRET_RADIAN)
        if target < -self.MAX_TURRET_COUNT:
            target += self.COUNTS_PER_TURRET_REV
        elif target > self.MAX_TURRET_COUNT:
            target += -self.COUNTS_PER_TURRET_REV
        self._slew_to_counts(target)

    def _slew_to_counts(self, counts: int) -> None:
        # Callers should ensure that the target is in range, but this is
        # defense in depth to not damage the robot.
        if counts < -self.MAX_TURRET_COUNT:
            counts = -self.MAX_TURRET_COUNT
        if counts > self.MAX_TURRET_COUNT:
            counts = self.MAX_TURRET_COUNT
        self.current_target_counts = counts

    def scan(self, azimuth=0.0) -> None:
        """
        Slew the turret back and forth looking for a target
        """
        # If we haven't hit an index yet, we just have to scan
        # about the current position.
        # Otherwise scan about the heading we've been given.
        # The target must be downfield from us, so scan up to
        # 90 degrees either side of the given heading

        if self.current_state != self.SCANNING:
            # First reset scan size
            self.current_scan_delta = self.scan_increment
            if self.index_found:
                # set the first pass
                self._slew_to_counts(
                    azimuth * self.COUNTS_PER_TURRET_RADIAN + self.scan_increment
                )
            else:
                current_count = self.motor.getSelectedSensorPosition()
                self._slew_to_counts(current_count + self.scan_increment)
            self.current_state = self.SCANNING

    def is_ready(self) -> bool:
        return (
            self.current_state != self.SCANNING
            and abs(self.motor.getClosedLoopError()) < self.ACCEPTABLE_ERROR_COUNTS
            and abs(self.motor.getSelectedSensorVelocity())
            < self.ACCEPTABLE_ERROR_SPEED
        )

    def execute(self) -> None:
        if not self.index_found:
            self._handle_indices()
        if self.current_state == self.SCANNING:
            self.motor.configMotionCruiseVelocity(self.SCAN_CRUISE_VELOCITY, 0)
            self._do_scanning()
        else:
            self.motor.configMotionCruiseVelocity(self.SLEW_CRUISE_VELOCITY, 0)

        self.motor.set(ctre.ControlMode.MotionMagic, self.current_target_counts)

        self.azimuth_history.appendleft(self.motor.getSelectedSensorPosition())

    def _handle_indices(self) -> None:
        # Check if we're at a known position
        # If so, update the encoder position on the motor controller
        # and change the current setpoint with the applied delta.
        # We do this only once, as indicated by the index_found flag.
        index = self._get_current_index()
        if index != Index.NOT_FOUND:
            count = self._index_to_counts(index)
            self._reset_encoder(count)
            self.index_found = True
            print("found an index and reset encoder")
        self.previous_index = index

    def _get_current_index(self) -> Index:
        if self.centre_index.get() == self.HALL_EFFECT_CLOSED:
            return Index.CENTRE
        if self.right_index.get() == self.HALL_EFFECT_CLOSED:
            return Index.RIGHT
        if self.left_index.get() == self.HALL_EFFECT_CLOSED:
            return Index.LEFT
        return Index.NOT_FOUND

    def _index_to_counts(self, index: Index) -> int:
        # We need to account for the width of the sensor.
        # This means we use the direction of travel to work out
        # which side we are approaching from
        # if we aren't moving at all, we assume we are in the middle
        count = self.INDEX_POSITIONS[index]
        velocity = self.motor.getSelectedSensorVelocity()
        if velocity != 0:
            direction = 1 if velocity > 0 else -1
            count += direction * self.HALL_EFFECT_HALF_WIDTH_COUNTS
        return count

    def _reset_encoder(self, counts) -> None:
        current_count = self.motor.getSelectedSensorPosition()
        delta = self.current_target_counts - current_count
        self.motor.setSelectedSensorPosition(counts)
        # Reset any current target using the new absolute azimuth
        for entry in self.azimuth_history:
            entry += current_count - counts
            # update old measurements
        self._slew_to_counts(counts + delta)

    def _do_scanning(self) -> None:
        # Check if we've finished a scan pass
        # If so, reverse the direction and increase pass size if necessary
        current_target = self.current_target_counts
        if (
            abs(self.motor.getSelectedSensorPosition() - self.current_target_counts)
            < self.ACCEPTABLE_ERROR_COUNTS
        ):
            current_target -= self.current_scan_delta
            if 0 < self.current_scan_delta < self.PI_OVER_4_IN_COUNTS:
                self.current_scan_delta = self.current_scan_delta + self.scan_increment
            if -self.PI_OVER_4_IN_COUNTS < self.current_scan_delta < 0:
                self.current_scan_delta = self.current_scan_delta - self.scan_increment
            self.current_scan_delta = -self.current_scan_delta
            current_target += self.current_scan_delta

        self._slew_to_counts(current_target)

    def azimuth_at_time(self, t: float) -> Optional[int]:
        """Get the stored azimuth (in radians) of the turret at a specified
        time. Returns None if the requested time is not in history
        @param t: time that we want data for
        """
        current_time = time.monotonic()
        control_loops_ago = int((current_time - t) / self.control_loop_wait_time)
        if control_loops_ago >= len(self.azimuth_history):
            return None
        return self.azimuth_history[control_loops_ago] / self.COUNTS_PER_TURRET_RADIAN

    def get_azimuth(self) -> int:
        """Get the current azimuth in radians"""
        return self.motor.getSelectedSensorPosition() / self.COUNTS_PER_TURRET_RADIAN
