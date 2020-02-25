import math
import time
from collections import deque
from enum import Enum

import wpilib
import ctre

from utilities.functions import constrain_angle


class Index(Enum):
    # These are relative to the turret, which faces backwards on the robot.
    NOT_FOUND = 0
    CENTRE = 1
    RIGHT = 2
    LEFT = 3


# Turret angles are rotated by 180 degrees (i.e. 0 is backwards on the robot).
ROBOT_TO_TURRET_OFFSET = math.pi

# Convert an angle in the robot coordinate system to the turret coordinate
# system.
def _robot_to_turret(angle: float) -> float:
    """
    Convert an angle in the robot coordinate system to the turret coordinate system.
    """
    return constrain_angle(angle - ROBOT_TO_TURRET_OFFSET)


# Convert an angle in the turret coordinate system to the robot coordinate
# system.
def _turret_to_robot(angle: float) -> float:
    """
    Convert an angle in the turret coordinate system to the robot coordinate system.
    """
    return constrain_angle(angle + ROBOT_TO_TURRET_OFFSET)


class Turret:
    #### Magicbot injections

    centre_index: wpilib.DigitalInput
    right_index: wpilib.DigitalInput
    left_index: wpilib.DigitalInput
    motor: ctre.WPI_TalonSRX

    MEMORY_CONSTANT: int
    control_loop_wait_time: float

    #### Constants

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
    # The following is used only if we don't start on an index
    DEFAULT_START_POSITION = INDEX_POSITIONS[Index.RIGHT]
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
    SCAN_INCREMENT = int(math.radians(10.0) * COUNTS_PER_TURRET_RADIAN)

    #### API

    def setup(self) -> None:
        self._setup_motor()
        self._setup_position()
        self.current_state = self.SLEWING

    def on_enable(self) -> None:
        self.motor.configPeakOutputForward(1.0, 10)
        self.motor.configPeakOutputReverse(-1.0, 10)
        if self.index_found:
            # Don't throw away previously found index
            self.motor.set(
                ctre.ControlMode.MotionMagic, self.motor.getSelectedSensorPosition()
            )
        else:
            index = self._get_current_index()
            if index is not Index.NOT_FOUND:
                # We are starting on an index
                self._handle_index(index)
            else:
                # We assume we are on the default start position.
                # We aren't actually there or there would be an index, but we
                # are probably close, or the robot hasn't been set up properly.
                self.motor.setSelectedSensorPosition(self.DEFAULT_START_POSITION)
                self.motor.set(ctre.ControlMode.MotionMagic, 0)
                self._enable_index_interrupts()

    def execute(self) -> None:
        if not self.index_found and self.index_hit != Index.NOT_FOUND:
            self._handle_index(self.index_hit)
        if self.current_state == self.SCANNING:
            self.motor.configMotionCruiseVelocity(self.SCAN_CRUISE_VELOCITY, 0)
            self._do_scanning()
        else:
            self.motor.configMotionCruiseVelocity(self.SLEW_CRUISE_VELOCITY, 0)

        self.motor.set(ctre.ControlMode.MotionMagic, self.current_target_counts)

        self.azimuth_history.appendleft(self.motor.getSelectedSensorPosition())

    # Slew to the given absolute angle in radians in the robot coordinate system.
    def slew_to_azimuth(self, angle: float) -> None:
        self.current_state = self.SLEWING
        turret_angle = _robot_to_turret(angle)
        self.motor._slew_to_counts(int(turret_angle * self.COUNTS_PER_TURRET_RADIAN))

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

    def scan(self, azimuth=0.0) -> None:
        """
        Slew the turret back and forth looking for a target
        """
        # The target must be downfield from us, so scan up to
        # 90 degrees either side of the given heading

        if self.current_state != self.SCANNING:
            # First reset scan size
            self.current_scan_delta = self.SCAN_INCREMENT
            turret_azimuth = _robot_to_turret(azimuth)
            # set the first pass
            self._slew_to_counts(
                turret_azimuth * self.COUNTS_PER_TURRET_RADIAN + self.SCAN_INCREMENT
            )
            self.current_state = self.SCANNING

    def is_ready(self) -> bool:
        return self.current_state != self.SCANNING and self._motor_is_finished()

    def azimuth_at_time(self, t: float) -> float:
        """Get the stored azimuth (in radians) of the turret at a specified
        time. Returns None if the requested time is not in history
        @param t: time that we want data for
        """
        current_time = time.monotonic()
        control_loops_ago = int((current_time - t) / self.control_loop_wait_time)
        if control_loops_ago >= len(self.azimuth_history):
            return (
                self._sensor_to_robot(self.azimuth_history[-1])
                if len(self.azimuth_history) > 0
                else self.get_azimuth()
            )
        return self._sensor_to_robot(self.azimuth_history[control_loops_ago])

    def get_azimuth(self) -> float:
        """Get the current azimuth in radians"""
        return self._sensor_to_robot(self.motor.getSelectedSensorPosition())

    #### Internal methods from here on

    #### Motor control, slewing, scanning

    def _setup_motor(self) -> None:
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

    def _slew_to_counts(self, counts: int) -> None:
        # Callers should ensure that the target is in range, but this is
        # defense in depth to not damage the robot.
        if counts < -self.MAX_TURRET_COUNT:
            counts = -self.MAX_TURRET_COUNT
        if counts > self.MAX_TURRET_COUNT:
            counts = self.MAX_TURRET_COUNT
        self.current_target_counts = counts

    def _motor_is_finished(self) -> bool:
        return (
            abs(self.motor.getClosedLoopError()) < self.ACCEPTABLE_ERROR_COUNTS
            and abs(self.motor.getSelectedSensorVelocity())
            < self.ACCEPTABLE_ERROR_SPEED
        )

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
                self.current_scan_delta = self.current_scan_delta + self.SCAN_INCREMENT
            if -self.PI_OVER_4_IN_COUNTS < self.current_scan_delta < 0:
                self.current_scan_delta = self.current_scan_delta - self.SCAN_INCREMENT
            self.current_scan_delta = -self.current_scan_delta
            current_target += self.current_scan_delta

        self._slew_to_counts(current_target)

    #### Position and Indexing
    # The turret always knows where it is. Until it encounters an index it
    # assumes it began at the DEFAULT_START_POSITION, which is -90 degrees in
    # the robot coordinate system, because that is where we must start to keep
    # the turret and motors from extending outside the frame perimeter.
    # An index is encountered either at start, if the turret was placed on one,
    # or by passing over one, which will trigger an interrupt. Once an index
    # been encountered, in either way, interrupts are disabled.

    def _setup_position(self) -> None:
        self.motor.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10
        )
        self.current_target_counts = 0
        self.azimuth_history = deque(maxlen=self.MEMORY_CONSTANT)
        self._setup_indexing()

    def _setup_indexing(self) -> None:
        self.index_found = False
        self.index_hit = Index.NOT_FOUND  # Interrupt will change this
        # set up interrupts on all three indices, on both edges, with separate
        # handlers for each DIO so we know which one triggered.
        self.centre_index.requestInterrupts(self._centre_isr)
        self.centre_index.setUpSourceEdge(True, True)
        self.left_index.requestInterrupts(self._left_isr)
        self.left_index.setUpSourceEdge(True, True)
        self.right_index.requestInterrupts(self._right_isr)
        self.right_index.setUpSourceEdge(True, True)

    def _enable_index_interrupts(self) -> None:
        self.centre_index.enableInterrupts()
        self.left_index.enableInterrupts()
        self.right_index.enableInterrupts()

    def _disable_index_interrupts(self) -> None:
        self.centre_index.disableInterrupts()
        self.left_index.disableInterrupts()
        self.right_index.disableInterrupts()

    def _centre_isr(
        self, wait_result: wpilib.InterruptableSensorBase.WaitResult
    ) -> None:
        self.index_hit = Index.CENTRE
        self._process_isr(wait_result)

    def _left_isr(self, wait_result: wpilib.InterruptableSensorBase.WaitResult) -> None:
        self.index_hit = Index.LEFT
        self._process_isr(wait_result)

    def _right_isr(
        self, wait_result: wpilib.InterruptableSensorBase.WaitResult
    ) -> None:
        self.index_hit = Index.RIGHT
        self._process_isr(wait_result)

    def _process_isr(
        self, wait_result: wpilib.InterruptableSensorBase.WaitResult
    ) -> None:
        self.index_count = self.motor.getSelectedSensorPosition()
        self.index_rising_edge = (
            wait_result is wpilib.InterruptableSensorBase.WaitResult.kRisingEdge
        )

    def _get_current_index(self) -> Index:
        if self.centre_index.get() == self.HALL_EFFECT_CLOSED:
            return Index.CENTRE
        if self.right_index.get() == self.HALL_EFFECT_CLOSED:
            return Index.RIGHT
        if self.left_index.get() == self.HALL_EFFECT_CLOSED:
            return Index.LEFT
        return Index.NOT_FOUND

    def _handle_index(self, index: Index) -> None:
        # Update the encoder position on the motor controller
        # and change the current setpoint with the applied delta.
        # We do this only once, as indicated by the index_found flag.
        count = self._index_to_counts(index)
        self._reset_encoder(count)
        self.index_found = True
        self._disable_index_interrupts()
        print("found an index and reset encoder")

    def _index_to_counts(self, index: Index) -> int:
        # We need to account for the width of the sensor.
        # This means we use the direction of travel and whether we interrupted
        # on the rising or falling edge.
        count = self.INDEX_POSITIONS[index]
        velocity = self.motor.getSelectedSensorVelocity()
        # if velocity is 0, assume the centre
        if velocity != 0:
            direction = 1 if velocity > 0 else -1
            # If it was the falling edge, then we want to adjust the count as
            # though it was the rising edge in the other direction.
            # If the turret is moving (velocity != 0), it isn't the start, so we
            # know we processed an interrupt and index_rising_edge will be set.
            if not self.index_rising_edge:
                direction = -direction
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

    def _sensor_to_robot(self, counts: int) -> float:
        return _turret_to_robot(counts / self.COUNTS_PER_TURRET_RADIAN)
