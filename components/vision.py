import time
import math

from networktables import NetworkTables, NetworkTable
from typing import Optional
from dataclasses import dataclass
from magicbot import feedback


@dataclass
class VisionData:
    #: The distance to the target in metres.
    #: Also used as the path type with balls vision
    distance: float

    #: The angle to the target in radians.
    angle: float

    #: An arbitrary timestamp, in seconds,
    #: for when the vision system last obtained data.
    timestamp: float

    __slots__ = ("distance", "angle", "timestamp")


class VisionComms:
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

        self.ping()

    def ping(self) -> None:
        """Send a ping to the RasPi to determine the connection latency."""
        self.ping_time_entry.setDouble(time.monotonic())

    def pong(self) -> None:
        """Receive a pong from the RasPi to determine the connection latency."""
        rio_pong_time = self.rio_pong_time_entry.getDouble(0)
        if abs(rio_pong_time - self.last_pong) > 1e-4:  # Floating point comparison
            raspi_pong_time = self.raspi_pong_time_entry.getDouble(0)
            alpha = 0.9  # Exponential averaging
            self.latency = (1 - alpha) * self.latency + alpha * (
                rio_pong_time - raspi_pong_time
            )
            self.last_pong = rio_pong_time

    def heart_beat(self) -> None:
        self.ping()
        self.pong()


class Vision:

    SYSTEM_LAG_THRESHOLD = 0.200

    def __init__(self) -> None:

        self.nt = NetworkTables
        self.table = self.nt.getTable("/vision")
        self.vision_data_entry = self.table.getEntry("data")

        self.visionComms = VisionComms(self.table)

        self.vision_data = None

    def get_data(self) -> Optional[VisionData]:
        """Returns the latest vision data.

        Returns None if there is no vision data.
        """
        return self.vision_data

    def execute(self) -> None:
        self.visionComms.heart_beat()
        data = None
        data = self.vision_data_entry.getDoubleArray(None)
        if data is not None:
            self.vision_data = VisionData(
                data[0], data[1], data[2] + self.visionComms.latency
            )
            # add latency to vision timestamp

        self.nt.flush()

    @feedback
    def is_ready(self) -> bool:
        return self.system_lag_calculation() < self.SYSTEM_LAG_THRESHOLD

    @feedback
    def system_lag_calculation(self) -> float:
        if self.vision_data is not None:
            return time.monotonic() - self.vision_data.timestamp
        else:
            return math.inf
