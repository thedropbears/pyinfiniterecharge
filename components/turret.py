from enum import Enum

import wpilib
import ctre
import math


class Index(Enum):
    NOT_FOUND = 0
    CENTRE = 1
    # RIGHT = 2
    # LEFT = 3
    # BACK = 4


class Turret:
    # TODO - There will be several (4 to 6) hall-effect sensors distributed
    # around the turrent ring, with one of them aligned with an azimuth of 0
    # relative to the robot. Right now there is only one hall-effect sensor at
    # that point.
    # left_index: wpilib.DigitalInput
    # right_index: wpilib.DigitalInput
    centre_index: wpilib.DigitalInput
    HALL_EFFECT_CLOSED = False

    motor: ctre.WPI_TalonSRX

    # Possible states
    IDLE = 0
    SLEWING = 1
    FINDING_INDEX = 2

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

    def __init__(self):
        # Note that we don't know where the turret actually is until we've
        # seen an index.
        self.current_azimuth: int
        self.current_state = self.IDLE

    def on_enable(self) -> None:
        self.motor.stopMotor()
        self.run_indexing()
        self.motor.setSelectedSensorPosition(0, 0)
        self.baseline_azimuth = self.motor.getSelectedSensorPosition(0)
        self.baseline_is_provisional = True
        self.current_azimuth = self.baseline_azimuth

    def setup(self) -> None:
        self.motor.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10
        )
        self.motor.config_kF(0, self.pidF, 10)
        self.motor.config_kP(0, self.pidP, 10)
        self.motor.config_kI(0, self.pidI, 10)
        self.motor.config_kD(0, self.pidD, 10)
        self.motor.configAllowableClosedloopError(0, self.CLOSED_LOOP_ERROR, 10)

    # Slew to the given absolute angle (in radians). An angle of 0 corresponds
    # to the centre index point.
    def slew_to_azimuth(self, angle: float) -> None:
        self._slew_to_count(
            angle * self.COUNTS_PER_TURRET_RADIAN - self.baseline_azimuth
        )

    # Slew the given angle (in radians) from the current position
    def slew(self, angle: float) -> None:
        self.current_azimuth = self.motor.getSelectedSensorPosition(0)
        self._slew_to_count(
            self.current_azimuth + angle * self.COUNTS_PER_TURRET_RADIAN
        )

    # Slew to the given absolute position, given as an encoder count
    def _slew_to_count(self, count: int) -> None:
        # Accept this only if we're not doing anything else
        if self.current_state == self.IDLE:
            # self.logger.info(f'slewing to count {count}')
            self.current_azimuth = self.motor.getSelectedSensorPosition(0)
            delta = count - self.current_azimuth
            self.target_count = self.current_azimuth + delta
            self.motor.set(ctre.ControlMode.Position, self.target_count)
            self.ticks_within_threshold = 0
            self.current_state = self.SLEWING

    def scan(self, heading):
        """
        Slew the turret back and forth looking for a target.
        """
        # If we haven't hit an index yet, we just have to scan
        # about the current position.
        # Otherwise scan about the heading we've been given.
        # The target must be downfield from us, so scan up to
        # 90 degrees either side of the given heading
        pass

    def is_ready(self) -> bool:
        # Once slewing or indexing is complete, state will go to IDLE
        return self.current_state == self.IDLE

    def execute(self) -> None:
        if self.current_state == self.FINDING_INDEX:
            self._do_indexing()
            return
        if self.current_state == self.SLEWING:
            self._do_slewing()
            return
        # state must be IDLE
        return

    # TODO: this currently uses only one hall-effect sensor; it should
    # use all of them.
    def _index_found(self) -> Index:
        if self.centre_index.get() == self.HALL_EFFECT_CLOSED:
            return Index.CENTRE
        # other sensors go here
        return Index.NOT_FOUND

    # Returns the encoder count corresponding to the given index value
    # TODO: implement
    def _index_to_count(self, index: Index) -> int:
        # TODO: implement for real
        if index == Index.CENTRE:
            return 0
        return 0

    # This is adapted from
    # https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html#mechanism-is-finished-command
    def _slewing_is_finished(self) -> bool:
        current_error = self.motor.getClosedLoopError()
        if -self.CLOSED_LOOP_ERROR < current_error < self.CLOSED_LOOP_ERROR:
            self.ticks_within_threshold += 1
        else:
            self.ticks_within_threshold = 0
        return self.ticks_within_threshold > self.TICKS_TO_SETTLE

    def _do_slewing(self):
        if self._slewing_is_finished():
            self.current_state = self.IDLE
            self.target_count = 0

    ################
    #
    # The remainder of this file, as well as the FINDING_INDEX state and all
    # the relevant references, can be removed once we no longer explicitly
    # do indexing.
    #
    ################

    # Seek for 200ms at first, before reversing and doubling
    STARTING_MAX_TICKS = 10

    # The indexing does not use the encoder at all right now. It just counts
    # the times it's called, incrementing (or decrementing) a tick count each
    # time. This method resets the counting mechanism.
    def _reset_ticks(self) -> None:
        self.motor_speed = 0.2  # Flips when seeking, so not a constant
        self.seeking = False
        self.tick_count = 0
        self.max_ticks = self.STARTING_MAX_TICKS
        self.max_ticks_factor = -2
        self.tick_increment = 1

    # Find the nearest index and reset the encoder
    def run_indexing(self) -> None:
        self._reset_ticks()
        self.current_state = self.FINDING_INDEX

    def _do_indexing(self):
        # Are we there yet? If so, greb the encoder value, stop the motor,
        # and stop seeking
        if self._index_found() == Index.CENTRE:
            self.baseline_azimuth = self.motor.getSelectedSensorPosition(0)
            self.baseline_is_provisional = False
            self.motor.stopMotor()
            self._reset_ticks()
            self.current_state = self.IDLE
            # self.logger.info("Found the sensor")
            return

        # If we haven't started yet, start seeking
        if not self.seeking:
            self.motor.set(ctre.ControlMode.PercentOutput, self.motor_speed)
            self.tick_count = 0
            self.max_ticks = self.STARTING_MAX_TICKS
            self.seeking = True
            # self.logger.info("starting to seek")
            return

        self.tick_count = self.tick_count + self.tick_increment
        if self.tick_count == self.max_ticks:
            self.motor.stopMotor()
            self.motor_speed = -self.motor_speed
            self.motor.set(ctre.ControlMode.PercentOutput, self.motor_speed)
            self.tick_increment = self.tick_increment * -1
            self.max_ticks = self.max_ticks * self.max_ticks_factor
            return

        # Currently running and haven't hit the limit nor found an index.
        # Just keep the motor running
        self.motor.set(ctre.ControlMode.PercentOutput, self.motor_speed)
