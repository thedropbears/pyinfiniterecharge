import math
from collections import deque
from enum import Enum
from typing import Optional

import wpilib
import ctre
import magicbot

from utilities.functions import constrain_angle


class Index(Enum):
    # These are relative to the turret, which faces backwards on the robot.
    NO_INDEX = 0
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

    # Directions for turret turnaround
    POSITIVE = 0
    NEGATIVE = 1

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
    pidP = 1.0
    pidI = 0.005
    pidIZone = 300
    pidD = 4.0
    SLEW_CRUISE_VELOCITY = 4000
    SCAN_CRUISE_VELOCITY = 1500
    CRUISE_ACCELERATION = int(SLEW_CRUISE_VELOCITY / 0.15)

    # Slew to within +- half a degree of the target azimuth. This is about
    # 50 encoder steps.
    ACCEPTABLE_ERROR_COUNTS = int(math.radians(0.5) * COUNTS_PER_TURRET_RADIAN)
    ACCEPTABLE_ERROR_SPEED = int(
        math.radians(0.5) * COUNTS_PER_TURRET_RADIAN
    )  # counts per 100ms

    PI_OVER_2_IN_COUNTS = int(math.pi / 2 * COUNTS_PER_TURRET_RADIAN)
    SCAN_INCREMENT = int(math.radians(20.0) * COUNTS_PER_TURRET_RADIAN)

    #### API

    index_found = magicbot.tunable(False)

    def setup(self) -> None:
        self._setup_motor()
        self._setup_position()
        self.current_state = self.SLEWING

    def __init__(self):
        self.disabled = False
        self.disable_when_done = False
        self.parked = False

    def on_enable(self) -> None:
        self.must_finish = False
        if self.index_found:
            # Don't throw away previously found index
            self.motor.set(
                ctre.ControlMode.MotionMagic, self.motor.getSelectedSensorPosition()
            )
        else:
            index = self._get_current_index()
            if index is not Index.NO_INDEX:
                # We are starting on an index
                self.index_count: Optional[int] = None
                self._handle_index(index)
            else:
                # We assume we are on the default start position.
                # We aren't actually there or there would be an index, but we
                # are probably close, or the robot hasn't been set up properly.
                self.motor.setSelectedSensorPosition(self.DEFAULT_START_POSITION)
                self.motor.set(ctre.ControlMode.MotionMagic, 0)
                self.index_hit = Index.NO_INDEX
                self._enable_index_interrupts()

    def on_disable(self) -> None:
        self.motor.stopMotor()

    def execute(self) -> None:
        if self.disable_when_done and self._motor_is_finished():
            self.disabled = True
            self.disable_when_done = False
            self.parked = True
        if self.disabled:
            self.motor.stopMotor()
            return
        if not self.index_found and self.index_hit != Index.NO_INDEX:
            self._handle_index(self.index_hit)
        if self.current_state == self.SCANNING:
            self.motor.configMotionCruiseVelocity(self.SCAN_CRUISE_VELOCITY, 0)
            self._do_scanning()
        else:
            self.motor.configMotionCruiseVelocity(self.SLEW_CRUISE_VELOCITY, 0)

        self.motor.set(ctre.ControlMode.MotionMagic, self.current_target_counts)

        self.azimuth_history.appendleft(self.motor.getSelectedSensorPosition())

        if self.must_finish and self._turnaround_sufficient():
            self.must_finish = False

    # Slew to the given absolute angle in radians in the robot coordinate system.
    def slew_to_azimuth(self, angle: float) -> None:
        if self.must_finish or self.disabled or self.disable_when_done:
            return
        self.current_state = self.SLEWING
        turret_angle = _robot_to_turret(angle)
        self._slew_to_counts(int(turret_angle * self.COUNTS_PER_TURRET_RADIAN))

    # Slew the given angle (in radians) from the current position
    def slew(self, angle: float) -> None:
        if self.must_finish or self.disabled or self.disable_when_done:
            return
        self.current_state = self.SLEWING
        current_pos = self.motor.getSelectedSensorPosition()
        target = current_pos + int(angle * self.COUNTS_PER_TURRET_RADIAN)
        if target < -self.MAX_TURRET_COUNT:
            target += self.COUNTS_PER_TURRET_REV
            self.must_finish = True
            self.turnaround_direction = self.POSITIVE
        elif target > self.MAX_TURRET_COUNT:
            target += -self.COUNTS_PER_TURRET_REV
            self.must_finish = True
            self.turnaround_direction = self.NEGATIVE
        self._slew_to_counts(target)

    def scan(self, azimuth=math.pi) -> None:
        """
        Slew the turret back and forth up to 90 degrees on either side, centred
        at the given azimuth.

        If the turret is already scanning, this adjusts the centre of the scan
        rather than starting over.
        """

        if self.must_finish or self.disabled or self.disable_when_done:
            return
        # If we aren't already scanning, reset scan size
        if self.current_state != self.SCANNING:
            self.current_scan_delta = self.SCAN_INCREMENT
        turret_azimuth = _robot_to_turret(azimuth)
        # If we are starting a new scan, the following is the first pass.
        # If we are adjusting an existing scan, this is the next adjusted pass.
        self._slew_to_counts(
            turret_azimuth * self.COUNTS_PER_TURRET_RADIAN + self.current_scan_delta
        )
        self.current_state = self.SCANNING

    def is_ready(self) -> bool:
        return self.current_state != self.SCANNING and self._motor_is_finished()

    def is_parked(self) -> bool:
        return self.parked

    def azimuth_at_time(self, t: float) -> float:
        """Get the stored azimuth (in radians) of the turret at a specified
        time. Returns the oldest data if the requested time is not in history
        @param t: time that we want data for
        """
        current_time = wpilib.Timer.getFPGATimestamp()
        control_loops_ago = int((current_time - t) / self.control_loop_wait_time)
        if control_loops_ago >= len(self.azimuth_history):
            return (
                self._sensor_to_robot(self.azimuth_history[-1])
                if len(self.azimuth_history) > 0
                else self.get_azimuth()
            )
        return self._sensor_to_robot(self.azimuth_history[control_loops_ago])

    def toggle(self):
        """Toggle the turret on and off"""
        self.disabled = not self.disabled

    @magicbot.feedback
    def get_azimuth(self) -> float:
        """Get the current azimuth in radians"""
        return self._sensor_to_robot(self.motor.getSelectedSensorPosition())

    def park_and_disable(self) -> None:
        self.slew_to_azimuth(0)
        self.disable_when_done = True

    #### Internal methods from here on

    #### Motor control, slewing, scanning

    def _setup_motor(self) -> None:
        self.motor.configFactoryDefault()

        # Positive motion is counterclockwise from above.
        self.motor.setInverted(ctre.InvertType.InvertMotorOutput)
        # set the peak and nominal outputs
        self.motor.configNominalOutputForward(0, 10)
        self.motor.configNominalOutputReverse(0, 10)
        self.motor.configPeakOutputForward(1.0, 10)
        self.motor.configPeakOutputReverse(-1.0, 10)
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
            abs(self.current_target_counts - self.motor.getSelectedSensorPosition())
            < self.ACCEPTABLE_ERROR_COUNTS
            and abs(self.motor.getSelectedSensorVelocity())
            < self.ACCEPTABLE_ERROR_SPEED
        )

    def _turnaround_sufficient(self) -> bool:
        if self.turnaround_direction == self.POSITIVE:
            if self.motor.getSelectedSensorPosition() > (
                self.current_target_counts - self.PI_OVER_2_IN_COUNTS
            ):
                return True
        else:
            if self.motor.getSelectedSensorPosition() < (
                self.current_target_counts + self.PI_OVER_2_IN_COUNTS
            ):
                return True
        return False

    def _do_scanning(self) -> None:
        # Check if we've finished a scan pass
        # If so, reverse the direction and increase pass size if necessary
        current_target = self.current_target_counts
        if self._motor_is_finished():
            current_target -= self.current_scan_delta
            if 0 < self.current_scan_delta < self.PI_OVER_2_IN_COUNTS:
                self.current_scan_delta = self.current_scan_delta + self.SCAN_INCREMENT
                if self.current_scan_delta > self.PI_OVER_2_IN_COUNTS:
                    self.current_scan_delta = self.PI_OVER_2_IN_COUNTS
            if -self.PI_OVER_2_IN_COUNTS < self.current_scan_delta < 0:
                self.current_scan_delta = self.current_scan_delta - self.SCAN_INCREMENT
                if self.current_scan_delta < -self.PI_OVER_2_IN_COUNTS:
                    self.current_scan_delta = -self.PI_OVER_2_IN_COUNTS
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
        self.index_hit = Index.NO_INDEX  # Interrupt will change this
        # set up interrupts on all three indices, on both edges, with separate
        # handlers for each DIO so we know which one triggered.
        self.centre_index.requestInterrupts(self._centre_isr)
        self.centre_index.setUpSourceEdge(True, False)
        self.left_index.requestInterrupts(self._left_isr)
        self.left_index.setUpSourceEdge(True, False)
        self.right_index.requestInterrupts(self._right_isr)
        self.right_index.setUpSourceEdge(True, False)

    def _enable_index_interrupts(self) -> None:
        self.centre_index.enableInterrupts()
        self.left_index.enableInterrupts()
        self.right_index.enableInterrupts()

    def _disable_index_interrupts(self) -> None:
        self.centre_index.disableInterrupts()
        self.left_index.disableInterrupts()
        self.right_index.disableInterrupts()

    def _centre_isr(self, _: wpilib.InterruptableSensorBase.WaitResult) -> None:
        self.index_hit = Index.CENTRE
        self.index_count = self.motor.getSelectedSensorPosition()

    def _left_isr(self, _: wpilib.InterruptableSensorBase.WaitResult) -> None:
        self.index_hit = Index.LEFT
        self.index_count = self.motor.getSelectedSensorPosition()

    def _right_isr(self, _: wpilib.InterruptableSensorBase.WaitResult) -> None:
        self.index_hit = Index.RIGHT
        self.index_count = self.motor.getSelectedSensorPosition()

    def _get_current_index(self) -> Index:
        if self.centre_index.get() == self.HALL_EFFECT_CLOSED:
            return Index.CENTRE
        if self.right_index.get() == self.HALL_EFFECT_CLOSED:
            return Index.RIGHT
        if self.left_index.get() == self.HALL_EFFECT_CLOSED:
            return Index.LEFT
        return Index.NO_INDEX

    def _handle_index(self, index: Index) -> None:
        # Update the encoder position on the motor controller
        # and change the current setpoint with the applied delta.
        # We do this only once, as indicated by the index_found flag.
        count = self._index_to_counts(index)
        self._reset_encoder(count)
        self.index_found = True
        self._disable_index_interrupts()
        print(f"found index {index} and reset encoder")

    def _index_to_counts(self, index: Index) -> int:
        # We need to account for the width of the sensor.
        # This means we use the direction of travel
        count = self.INDEX_POSITIONS[index]
        velocity = self.motor.getSelectedSensorVelocity()
        print(f"velocity when sensor hit {velocity}")
        # if velocity is 0, assume the centre
        if velocity != 0:
            direction = 1 if velocity > 0 else -1
            count += direction * self.HALL_EFFECT_HALF_WIDTH_COUNTS
        return count

    def _reset_encoder(self, counts) -> None:
        current_count = self.motor.getSelectedSensorPosition()
        # delta = self.current_target_counts - current_count
        delta = 0
        index_to_now = 0
        if self.index_count is not None:
            index_to_now = current_count - self.index_count
            delta = self.index_count - counts
        self.motor.setSelectedSensorPosition(counts + index_to_now, timeoutMs=5)
        print(f"========= resetting encoder to {counts} + {index_to_now} ==========")
        # Reset any current target using the new absolute azimuth
        for i, entry in enumerate(self.azimuth_history):
            self.azimuth_history[i] = entry - delta
            # update old measurements
        if self.current_state == self.SLEWING:
            print("state is SLEWING, so adjusting slew target")
            self._slew_to_counts(self.current_target_counts - delta)
        print(f"moved command from {self.current_target_counts} by {delta}")

    def _sensor_to_robot(self, counts: int) -> float:
        return _turret_to_robot(counts / self.COUNTS_PER_TURRET_RADIAN)
