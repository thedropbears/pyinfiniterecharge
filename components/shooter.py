import time

import wpilib
import rev
from numpy import interp
from magicbot import tunable


class Shooter:
    outer_motor: rev.CANSparkMax
    centre_motor: rev.CANSparkMax
    loading_piston: wpilib.Solenoid

    ranges = (7, 8, 9, 10, 11)
    centre_rpms = (880, 1120, 1500, 2150, 2400)

    # outer_rpm = tunable(0)
    # centre_rpm = tunable(0)
    def __init__(self):
        self.outer_rpm = 4400
        self.centre_rpm = -3700

        self.inject = False
        self.velocity_tolerance = 50 # rpm

    def on_enable(self) -> None:
        self.centre_motor.stopMotor()
        self.outer_motor.stopMotor()

    def setup(self) -> None:
        self.loading_piston.setPulseDuration(0.5)
        self.outer_motor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        self.centre_motor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)

        self.outer_encoder = self.outer_motor.getEncoder()
        self.centre_encoder = self.centre_motor.getEncoder()

        self.centre_pid = self.centre_motor.getPIDController()
        self.outer_pid = self.outer_motor.getPIDController()

        for pid in (self.centre_pid, self.outer_pid):
            pid.setP(5e-5)
            pid.setI(1e-6)
            pid.setD(0)
            pid.setFF(0.000156)

    def execute(self) -> None:
        self.centre_pid.setReference(self.centre_rpm, rev.ControlType.kVelocity)
        self.outer_pid.setReference(self.outer_rpm, rev.ControlType.kVelocity)

        if self.inject:
            self.loading_piston.startPulse()
            self.inject = False
    
    def set_range(self, dist: float) -> None:
        """
        Set the target range for the shooter, this will be converted into target speeds for the flywheels
        dist: planar distance from the power port
        """
        if 7 <= dist <= 11:
            self.centre_rpm = interp(dist, self.ranges, self.centre_rpms)
            self.outer_rpm = 5000

    def is_ready(self) -> bool:
        """
        Returns true if the shooter is able to make a successful shot with the currently set dist.

        Considers the rotation rates of the flywheels compared with their setpoints
        """
        return (abs(self.centre_rpm - self.centre_encoder.getVelocity()) <= self.velocity_tolerance 
                and abs(self.outer_rpm - self.outer_encoder.getVelocity() <= self.velocity_tolerance))

    def is_firing(self) -> bool:
        """
        Returns true if the shooter is attempting a shot.

        based off of the pistons current state
        """
        return not self.loading_piston.get()



    def fire(self) -> None:
        """
        Inject a ball into the shooter
        """
        self.inject = True

    # def get_centre_error(self) -> float:
    #     return self.centre_rpm - self.centre_encoder.getVelocity()

    # def get_outer_error(self) -> float:
    #     return self.outer_rpm - self.outer_encoder.getVelocity()

    # def set_motor_rpm(self, v_outer, v_centre) -> None:
    #     self.centre_rpm = v_centre
    #     self.outer_rpm = v_outer
