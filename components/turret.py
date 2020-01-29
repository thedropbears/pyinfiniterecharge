import wpilib
import ctre
import math


class Turret:
    # TODO - There should be 5 indexes total: left, right, centre, and two
    # limit switches. Right now there is only one in the centre.
    # left_index: wpilib.DigitalInput
    # right_index: wpilib.DigitalInput
    centre_index: wpilib.DigitalInput

    joystick: wpilib.Joystick

    motor: ctre.WPI_TalonSRX

    HALL_EFFECT_CLOSED = 0
    # Consider inverting the input instead

    # Possible states
    IDLE = 0
    SLEWING = 1
    FINDING_INDEX = 2

    # Constants for Talon on the turret
    COUNTS_PER_MOTOR_REV = 4096
    GEAR_REDUCTION = 160 / 12
    COUNTS_PER_TURRET_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION
    COUNTS_PER_TURRET_RADIAN = COUNTS_PER_TURRET_REV / math.tau

    # Seek for 200ms at first, before reversing and doubling
    STARTING_MAX_TICKS = 10

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
        err = self.motor.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10
        )
        if err != ctre.ErrorCode.OK:
            self.logger.warning(f"Error configuring encoder: {err}")

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
        self.current_azimuth = self.motor.getSelectedSensorPosition(0)
        delta = count - self.current_azimuth
        self.target_count = self.current_azimuth + delta
        self.incrementing = True
        if self.target_count < self.current_azimuth:
            self.incrementing = False
        self.current_state = self.SLEWING

    # Find the nearest index and reset the encoder
    def run_indexing(self) -> None:
        self.current_state = self.FINDING_INDEX

    def is_ready(self) -> bool:
        if self.current_state == self.IDLE:
            return True
        if self.current_state == self.SLEWING:
            self.current_azimuth = self.motor.getSelectedSensorPosition(0)
            if (self.incrementing and self.current_azimuth >= self.target_count) or (
                not self.incrementing and self.current_azimuth <= self.target_count
            ):
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

    def _do_indexing(self):
        # TODO: this currently uses only the centre index; it should use
        # All three hall-effect sensors and the two limit switches
        # Are we there yet? If so, greb the encoder value, stop the motor,
        # and stop seeking
        val = self.centre_index.get()
        if val == self.HALL_EFFECT_CLOSED:
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
            self.motor.stopMotor()
            self.current_state = self.IDLE
            self.target_count = 0
            self.tick_count = 0
            return

        # Not there, so keep the motor running
        speed = self.motor_speed
        if self.target_count < self.current_azimuth:
            speed = -speed
        self.motor.set(ctre.ControlMode.PercentOutput, speed)
