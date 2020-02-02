from networktables import NetworkTables
from typing import Tuple
import time
import math


class Vision:
    def __init__(self) -> None:
        self.nt = NetworkTables
        self.table = self.nt.getTable("/vision")
        self.entry = self.table.getEntry("data")
        self.lastTime = 0
        self.repeats = 0

    def get_vision_data(self) -> Tuple[float, float, float]:
        """Returns a tuple containing the distance (metres), angle (radians), and timestamp (time.monotonic)
        If it can't get info, it returns [None, None, None]
        """
        self.data = self.entry.getDoubleArray([None, None, None])
        return self.data

    def is_ready(self) -> int:
        if self.data[2] == self.lastTime:  # if it gets the same time twice
            self.repeats += 1
        else:
            self.repeats = 0
        self.lastTime = self.data[2]

        if self.repeats >= 5:  # if it has got the same data for five loops
            return 0  # no target
        if abs(self.data[1]) < math.degrees(5):
            return 1  # target out of alignment
        else:
            return 2  # aligned
