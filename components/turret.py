import wpilib
import ctre
import magicbot
import math

class Turret:
    left_index: wpilib.DigitalInput
    centre_index: wpilib.DigitalInput
    right_index: wpilib.DigitalInput
    joystick: wpilib.Joystick
    
    HALL_EFFECT_CLOSED = 0; # Consider inverting the input instead
    
    # Possible states
    IDLE = 0
    SLEWING = 1
    FINDING_INDEX = 2
    
    # Constants for Talon on the turret
    COUNTS_PER_MOTOR_REV = 4096
    GEAR_REDUCTION = 160/12
    COUNTS_PER_TURRET_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION
    COUNTS_PER_TURRET_RADIAN = COUNTS_PER_TURRET_REV / math.tau
        
    def __init__(self):
        self.motor = ctre.WPI_TalonSRX(10)            
        # Note that we don't know where the turret actually is.
        self.current_azimuth: int # in encoder counts off the centre_index
        # TODO: do we need to invert the encoder?

    def reset_ticks(self) -> None:
        self.motor_speed = 0.2
        self.seeking = False
        self.tick_count = 0
        self.starting_max_ticks = 10
        self.max_ticks = self.starting_max_ticks
        self.max_ticks_factor = -2
        self.tick_increment = 1

    def on_enable(self) -> None:
        self.motor.stopMotor()
        self.reset_ticks()
        self.current_state = self.FINDING_INDEX 

    def setup(self) -> None:
        err = self.motor.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10
        ) 
        if err != ctre.ErrorCode.OK:
            self.logger.error(f'Error configuring encoder: {err}')
            raise EnvironmentError("Couldn't configure encoder")

    # Slew to the given angle (in radians) off the baseline.
    # Calls callback when done
    def slew_to_azimuth(self, angle: float, callback) -> None:
        self.target_count = angle * self.COUNTS_PER_TURRET_RADIAN
        self.start_slew(callback)
        
    # adjust relative angle (in radians)
    # Calls callback when done
    def slew(self, angle: float, callback) -> None:
        delta = angle * self.COUNTS_PER_TURRET_RADIAN
        self.target_count = self.current_azimuth + delta
        self.logger.info(f'slewing delta: {delta}, current_azimuth: {self.current_azimuth}')
        self.start_slew(callback)

    def start_slew(self, callback):
        self.callback = callback
        self.incrementing = True
        if self.target_count < self.current_azimuth:
            self.incrementing = False
        self.logger.info(f'starting slewing target count: {self.target_count}, incrementing: {self.incrementing}')
        self.current_state = self.SLEWING
        
    # Find the nearest index and reset the encoder
    def run_indexing(self, callback) -> None:
        self.callback = callback
        self.current_state = self.FINDING_INDEX

    def execute(self) -> None:
        if self.current_state == self.FINDING_INDEX:
            # TODO: this currently uses only the centre index; it should use
            # All three hall-effect sensors and the two limit switches
            # Are we there yet? If so, stop the motor and stop seeking
            val = self.centre_index.get()
            if val == self.HALL_EFFECT_CLOSED:
                self.logger.info("Yep, we are there!!!")
                self.motor.stopMotor()
                self.reset_ticks()
                self.current_azimuth = 0
                err = self.motor.setSelectedSensorPosition(0)
                if err != ctre.ErrorCode.OK:
                    self.logger.error(f'Error zeroing encoder: {err}')
                    raise EnvironmentError("Couldn't zero encoder")
                self.current_state = self.IDLE
                if hasattr(self, "callback"):
                    self.callback()
                return
            
            # Start seeking right
            if self.seeking == False:
                self.logger.info("starting to seek")
                self.motor.set(ctre.ControlMode.PercentOutput, self.motor_speed)
                self.tick_count = 0
                self.max_ticks = self.starting_max_ticks
                self.seeking = True
                return
            
            self.tick_count = self.tick_count + self.tick_increment
            if self.tick_count == self.max_ticks:
                self.logger.info("hitting the limit, turning around")
                self.motor.stopMotor()
                self.motor_speed = -self.motor_speed
                self.motor.set(ctre.ControlMode.PercentOutput, self.motor_speed)
                self.tick_increment = self.tick_increment * -1
                self.max_ticks = self.max_ticks * self.max_ticks_factor
                return
                
            # Just keep the motor running
            self.motor.set(ctre.ControlMode.PercentOutput, self.motor_speed)
        
        if self.current_state == self.SLEWING:
            # Are we there yet?
            self.current_azimuth = self.motor.getSelectedSensorPosition(0)
            self.tick_count += 1
            if self.tick_count % 25 == 0:
                self.logger.info(f'current sensor reading: {self.current_azimuth}')
            if ((self.incrementing and self.current_azimuth >= self.target_count) or
            (not self.incrementing and self.current_azimuth <= self.target_count)):
                self.motor.stopMotor()
                self.current_state = self.IDLE
                self.target_count = 0
                self.tick_count = 0
                self.logger.info("Done slewing. We are there!!!")
                self.callback()
                return
            
            # Not there, so keep the motor running
            speed = self.motor_speed
            if self.target_count < self.current_azimuth:
                speed = -speed
            self.motor.set(ctre.ControlMode.PercentOutput, speed)

        
