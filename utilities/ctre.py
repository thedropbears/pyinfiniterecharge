import ctre
import wpilib


class TalonSRXEncoder(wpilib.Sendable):
    """Wrapper around a Talon SRX selected sensor, mimicking wpilib.Encoder."""

    __slots__ = ("talon", "pid_loop_idx", "distance_per_pulse")

    def __init__(
        self, talon: ctre.TalonSRX, pid_loop_idx: int = 0, distance_per_pulse: float = 1
    ):
        super().__init__()
        self.talon = talon
        self.pid_loop_idx = pid_loop_idx
        self.distance_per_pulse = distance_per_pulse
        wpilib.SendableRegistry.getInstance().add(
            self, "TalonSRX Encoder", talon.getDeviceID(), pid_loop_idx
        )

    def getDistancePerPulse(self) -> float:
        return self.distance_per_pulse

    def getRaw(self) -> float:
        """Get the raw number of counts from the encoder."""
        return self.talon.getSelectedSensorPosition(self.pid_loop_idx)

    def reset(self) -> None:
        """Reset the encoder distance to 0, iff it is a relative encoder."""
        self.talon.setSelectedSensorPosition(0, self.pid_loop_idx)

    def getDistance(self) -> float:
        """Get the distance driven as scaled by :meth:`setDistancePerPulse`."""
        return (
            self.talon.getSelectedSensorPosition(self.pid_loop_idx)
            * self.distance_per_pulse
        )

    def getRate(self) -> float:
        """Get the current rate of the encoder.
        Units are distance per second as scaled by :meth:`setDistancePerPulse`.
        """
        return (
            self.talon.getSelectedSensorVelocity(self.pid_loop_idx)
            * self.distance_per_pulse
            * 10
        )

    def initSendable(self, builder: wpilib.SendableBuilder) -> None:
        builder.setSmartDashboardType("Encoder")
        builder.addDoubleProperty("Distance", self.getDistance, None)
        builder.addDoubleProperty("Speed", self.getRate, None)
        builder.addDoubleProperty("Distance per tick", self.getDistancePerPulse, None)
