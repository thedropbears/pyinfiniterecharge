import wpilib
import ctre
import math


class Turret:
    # TODO - There should be 5 indexes total: left, right, and centre hall-effect
    # sensors, and two limit switches. Right now there is only one hall-effect
    # sensor in the centre.
    # left_index: wpilib.DigitalInput
    # right_index: wpilib.DigitalInput
    centre_index: wpilib.DigitalInput
    INDEX_NOT_FOUND = 0
    INDEX_CENTRE = 1
    # INDEX_RIGHT = 2
    # INDEX_LEFT_LIMIT = 3
    # INDEX_RIGHT_LIMIT = 4
    HALL_EFFECT_CLOSED = False

    joystick: wpilib.Joystick

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

    # Seek for 200ms at first, before reversing and doubling
    STARTING_MAX_TICKS = 10

    # PID values
    pidF = 0
    pidP = 0.2
    pidI = 0
    pidD = 0

    # Arbitrarily start at 5 degrees. TODO: This is way too big to aim. We
    # currently believe this is necessary due to the large amount of jitter in
    # in the angle returned from vision.
    MIN_CLOSED_LOOP_ERROR = 5 * math.pi / 180 * COUNTS_PER_TURRET_RADIAN

    def __init__(self):
        # Note that we don't know where the turret actually is until we've
        # run the indexing.
        self.current_azimuth: int
        # TODO: do we need to invert the encoder?

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

    def on_enable(self) -> None:
        self.motor.stopMotor()
        self._reset_ticks()
        self.run_indexing()

    def setup(self) -> None:
        # self.motor.configFactoryDefault()
        err = self.motor.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10
        )
        if err != ctre.ErrorCode.OK:
            self.logger.warning(f"Error configuring encoder: {err}")
        self.motor.config_kF(0, self.pidF, 10)
        self.motor.config_kP(0, self.pidP, 10)
        self.motor.config_kI(0, self.pidI, 10)
        self.motor.config_kD(0, self.pidD, 10)

    # Slew to the given absolute angle (in radians). An angle of 0 corresponds
    # to the centre index point.
    def slew_to_azimuth(self, angle: float) -> None:
        if self.current_state != self.FINDING_INDEX:
            self._slew_to_count(
                angle * self.COUNTS_PER_TURRET_RADIAN - self.baseline_azimuth
            )

    # Slew the given angle (in radians) from the current position
    def slew(self, angle: float) -> None:
        if self.current_state != self.FINDING_INDEX:
            self.current_azimuth = self.motor.getSelectedSensorPosition(0)
            self._slew_to_count(
                self.current_azimuth + angle * self.COUNTS_PER_TURRET_RADIAN
            )

    # Slew to the given absolute position, given as an encoder count
    # This should change to use the Talon Absolute Position mode
    def _slew_to_count(self, count: int) -> None:
        #self.logger.info(f'slewing to count {count}')
        self.current_azimuth = self.motor.getSelectedSensorPosition(0)
        delta = count - self.current_azimuth
        self.target_count = self.current_azimuth + delta
        # self.incrementing = True
        # if self.target_count < self.current_azimuth:
        #    self.incrementing = False
        #self.logger.info(f'calling motor to go from count {self.current_azimuth} to count {self.target_count}')
        self.motor.set(ctre.ControlMode.Position, self.target_count)
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

    # Find the nearest index and reset the encoder
    def run_indexing(self) -> None:
        self.current_state = self.FINDING_INDEX

    def is_ready(self) -> bool:
        if self.current_state == self.IDLE:
            # self.logger.info("is_ready in IDLE state")
            return True
        if self.current_state == self.SLEWING:
            # self.current_azimuth = self.motor.getSelectedSensorPosition(0)
            # if (self.incrementing and self.current_azimuth >= self.target_count) or (
            #    not self.incrementing and self.current_azimuth <= self.target_count
            # ):
            closed_loop_error = self.motor.getClosedLoopError(0)
            #self.logger.info(f'is_ready check: error i {closed_loop_error}, min is {self.MIN_CLOSED_LOOP_ERROR}')
            if abs(closed_loop_error) < self.MIN_CLOSED_LOOP_ERROR:
                return True
            else:
                return False
        # state must be FINDING_INDEX. Return False until it's done.
        return False

    def execute(self) -> None:
        if self.current_state == self.FINDING_INDEX:
            self._do_indexing()
            return
        if self.current_state == self.SLEWING:
            self._do_slewing()
            return
        # state must be IDLE
        return

    # TODO: this currently uses only the centre hall-effect sensor; it should
    # use all three hall-effect sensors and the two limit switches.
    def _index_found(self) -> int:
        if self.centre_index.get() == self.HALL_EFFECT_CLOSED:
            return self.INDEX_CENTRE
        # other sensors go here
        return self.INDEX_NOT_FOUND

    def _do_indexing(self):
        # Are we there yet? If so, greb the encoder value, stop the motor,
        # and stop seeking
        if self._index_found() == self.INDEX_CENTRE:
            self.baseline_azimuth = self.motor.getSelectedSensorPosition(0)
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

    def _do_slewing(self):
        # The following will have to change to use the Talon Absolute Position mode
        # Are we there yet?
        if self.is_ready():
            self.logger.info("Hey, I'm ready!")
            self.motor.stopMotor()
            self.current_state = self.IDLE
            self.target_count = 0
            self.tick_count = 0
            return

        # Not there, so keep the motor running
        # speed = self.motor_speed
        # if self.target_count < self.current_azimuth:
        #    speed = -speed
        # self.motor.set(ctre.ControlMode.PercentOutput, speed)
