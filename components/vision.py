from networktables import NetworkTables
from typing import Optional
from dataclasses import dataclass


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
        self.vision_data_entry = self.table.getEntry("data")
        self.lastTime = 0
        self.repeats = 0
        self.data = None

    def get_data(self) -> Optional[VisionData]:
        """Returns the latest vision data.

        Returns None if there is no vision data.
         """
        return self.data

    def execute(self) -> None:
        data = None
        data = self.vision_data_entry.getDoubleArray(None)
        if data is not None:
            self.data = VisionData(*data)

    def is_ready(self) -> bool:
        if self.vision_data_entry.exists() and self.data[2] != None:
            if self.data[2] == self.lastTime:  # if it gets the same time twice
                self.repeats += 1
            else:
                self.repeats = 0
            self.lastTime = self.data[2]

            if self.repeats > 5:  # if it has got the same data for five loops
                return False  # no target
            else:
                return True  # has a target
        return False  # no network tables, so no target
