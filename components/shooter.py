import wpilib
import rev
from numpy import interp
from magicbot import feedback, tunable


class Shooter:
    outer_motor: rev.CANSparkMax
    centre_motor: rev.CANSparkMax
    loading_piston: wpilib.Solenoid

    led: wpilib.AddressableLED

    ranges = (0, 7, 8, 9, 10, 11)  # TODO remove 0 and add more data points
    centre_rpms = (0, 880, 1120, 1500, 2150, 2400)

    outer_rpm = tunable(0)
    centre_rpm = tunable(0)

    def __init__(self):
        self.inject = False
        self.in_range = False
        self.velocity_tolerance = 0.05  # of setpoint

    def on_enable(self) -> None:
        self.centre_motor.stopMotor()
        self.outer_motor.stopMotor()

    def setup(self) -> None:
        self.loading_piston.setPulseDuration(0.5)
        self.outer_motor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        self.centre_motor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)

        self.outer_motor.setInverted(True)
        self.centre_motor.setInverted(False)

        self.outer_encoder = self.outer_motor.getEncoder()
        self.centre_encoder = self.centre_motor.getEncoder()

        self.centre_pid = self.centre_motor.getPIDController()
        self.outer_pid = self.outer_motor.getPIDController()

        self.outer_pid.setP(0.958 / 3600)
        self.outer_pid.setI(1e-7)
        self.outer_pid.setD(0)
        self.outer_pid.setFF(0.000156)
        self.centre_pid.setP(0.959 / 3600)
        self.centre_pid.setI(1e-7)
        self.centre_pid.setD(0)
        self.centre_pid.setFF(0.000156)

        self.led_length = 72
        self.led_speed = wpilib._wpilib.AddressableLED.LEDData(255, 0, 0)
        self.led_ball = wpilib._wpilib.AddressableLED.LEDData(0, 255, 0)
        self.led_vision = wpilib._wpilib.AddressableLED.LEDData(0, 0, 255)
        self.led.setLength(self.led_length)
        self.led_vals = (
            [self.led_speed] * int(self.led_length / 3)
            + [self.led_ball] * int(self.led_length / 3)
            + [self.led_vision] * int(self.led_length / 3)
        )
        self.led.setData(self.led_vals)
        self.led.start()

    def execute(self) -> None:
        # self.centre_rpm = 2000
        # self.outer_rpm = 5000
        self.centre_pid.setReference(self.centre_rpm, rev.ControlType.kVelocity)
        self.outer_pid.setReference(self.outer_rpm, rev.ControlType.kVelocity)

        if self.inject:
            self.loading_piston.startPulse()
            self.inject = False

        self.led.setData(self.led_vals)

    def set_flywheel_led(self, r, g, b) -> None:
        self.led_speed.setRGB(r, g, b)

    def set_ball_ready_led(self, r, g, b) -> None:
        self.led_ball.setRGB(r, g, b)

    def set_vision_led(self, r, g, b) -> None:
        self.led_vision.setRGB(r, g, b)

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
        self.centre_rpm = interp(dist, self.ranges, self.centre_rpms)
        self.outer_rpm = 5000

    @feedback
    def is_at_speed(self) -> bool:
        """
        Returns true if the shooter is spinning at the set speed.

        Considers the rotation rates of the flywheels compared with their setpoints
        """
        return (
            abs(self.centre_rpm - self.centre_encoder.getVelocity())
            <= self.centre_rpm * self.velocity_tolerance
            and abs(self.outer_rpm - self.outer_encoder.getVelocity())
            <= self.outer_rpm * self.velocity_tolerance
        )

    @feedback
    def get_centre_velocity(self):
        return self.centre_encoder.getVelocity()

    @feedback
    def get_outer_velocity(self):
        return self.outer_encoder.getVelocity()

    @feedback
    def is_firing(self) -> bool:
        """
        Returns true if the shooter is attempting a shot.

        based off of the pistons current state
        """
        return self.loading_piston.get()

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
