import ctre
import hal


class TalonEncoder:
    """Wrapper around a Talon SRX/FX selected sensor."""

    # TODO: wpilib.Sendable (probably 2021)

    __slots__ = (
        "talon",
        "distance_per_edge",
        "pid_loop_idx",
        "sim_dev",
        "sim_pos",
        "sim_vel",
    )

    def __init__(
        self,
        talon: ctre.BaseTalon,
        distance_per_edge: float = 1,
        pid_loop_idx: int = 0,
    ) -> None:
        """
        Args:
            talon: The TalonSRX or TalonFX to read from.
            distance_per_edge: Conversion factor from edges to useful units.
            pid_loop_idx: The PID loop index for the sensor on the Talon.
        """
        super().__init__()

        self.sim_dev = hal.SimDevice("TalonEncoder", talon.getDeviceID(), pid_loop_idx)
        if self.sim_dev:
            self.sim_pos = self.sim_dev.createDouble("Position", False, 0)
            self.sim_vel = self.sim_dev.createDouble("Velocity", False, 0)
            self.sim_dev.createDouble("ConversionFactor", True, distance_per_edge)

        self.talon = talon
        self.distance_per_edge = distance_per_edge
        self.pid_loop_idx = pid_loop_idx

    def getPosition(self) -> float:
        """Get the position of the sensor in useful units."""
        if self.sim_dev:
            return self.sim_pos.get()
        return (
            self.talon.getSelectedSensorPosition(self.pid_loop_idx)
            * self.distance_per_edge
        )

    def getVelocity(self) -> float:
        """Get the velocity of the sensor in useful units per second."""
        if self.sim_dev:
            return self.sim_vel.get()
        return (
            self.talon.getSelectedSensorVelocity(self.pid_loop_idx)
            * self.distance_per_edge
            * 10
        )
