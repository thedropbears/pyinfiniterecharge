from dataclasses import dataclass
from typing import Optional

from networktables import NetworkTables


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


class Vision:
    def __init__(self) -> None:
        self.nt = NetworkTables
        self.table = self.nt.getTable("/vision")
        self.entry = self.table.getEntry("data")

    def get_data(self) -> Optional[VisionData]:
        """Returns the latest vision data.

        Returns None if there is no vision data.
        """
        data = self.entry.getDoubleArray(None)
        if data is not None:
            return VisionData(*data)
