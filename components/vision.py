import time

from networktables import NetworkTables, NetworkTable
from typing import Optional
from dataclasses import dataclass
from magicbot import feedback


@dataclass
class VisionData:
    #: The distance to the target in metres.
    distance: float

    #: The angle to the target in radians.
    angle: float

    #: An arbitrary timestamp, in seconds,
    #: for when the vision system last obtained data.
    timestamp: float

    __slots__ = ("distance", "angle", "timestamp")


class VisionComms:
    @property
    def ping_time(self) -> float:
        return self.ping_time_entry.getDouble(0.0)

    @ping_time.setter
    def ping_time(self, value: float) -> None:
        self.ping_time_entry.setDouble(value)

    @property
    def rio_pong_time(self) -> float:
        return self.rio_pong_time_entry.getDouble(0.0)

    @property
    def raspi_pong_time(self) -> float:
        return self.raspi_pong_time_entry.getDouble(0.0)

    @property
    def latency(self) -> float:
        return self.latency_entry.getDouble(0.0)

    @latency.setter
    def latency(self, value: float) -> None:
        self.latency_entry.setDouble(value)

    def __init__(self, table: NetworkTable) -> None:
        self.last_pong = time.monotonic()

        self.table = table
        self.ping_time_entry = self.table.getEntry("ping")
        self.rio_pong_time_entry = self.table.getEntry("rio_pong")
        self.raspi_pong_time_entry = self.table.getEntry("raspi_pong")
        self.latency_entry = self.table.getEntry("clock_offset")
        self.ping_time = time.monotonic()

    def ping(self) -> None:
        """Send a ping to the RasPi to determine the connection latency."""
        self.ping_time = time.monotonic()

    def pong(self) -> None:
        """Receive a pong from the RasPi to determine the connection latency."""
        if abs(self.rio_pong_time - self.last_pong) > 1e-4:  # Floating point comparison
            alpha = 0.9  # Exponential averaging
            self.latency = (1 - alpha) * self.latency + alpha * (
                self.rio_pong_time - self.raspi_pong_time
            )
            self.last_pong = self.rio_pong_time

    def heart_beat(self) -> None:
        if self.ping_time_entry.exists():
            self.ping()
            self.pong()


class Vision:
    def __init__(self) -> None:

        self.nt = NetworkTables
        self.table = self.nt.getTable("/vision")
        self.vision_data_entry = self.table.getEntry("data")

        self.visionComms = VisionComms(self.table)

        self.data: Optional[VisionData] = None

    def get_data(self) -> Optional[VisionData]:
        """Returns the latest vision data.

        Returns None if there is no vision data.
         """
        return self.data

    def execute(self) -> None:
        self.visionComms.heart_beat()
        data = self.vision_data_entry.getDoubleArray(None)
        if data is not None:
            self.data = VisionData(*data)

        self.nt.flush()

    @property
    def target_in_sight(self) -> bool:
        return (
            self.data is not None
            and time.monotonic() - (self.data.timestamp + self.visionComms.latency)
            < 0.1
        )

    @feedback
    def is_ready(self) -> bool:
        if self.vision_data_entry.exists() and self.target_in_sight:
            return True
        return False  # no network tables, so no target
