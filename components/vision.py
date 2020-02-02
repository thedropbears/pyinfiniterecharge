from networktables import NetworkTables
from typing import Tuple


class Vision:
    def __init__(self) -> None:
        self.nt = NetworkTables
        self.table = self.nt.getTable("/vision")
        self.entry = self.table.getEntry("data")
        self.lastTime = 0
        self.repeats = 0
        self.data = [None, None, None]

    def get_vision_data(self) -> Tuple[float, float, float]:
        """Returns a tuple containing the distance (metres), angle (radians), and timestamp (time.monotonic)
        If it can't get the entry, it returns [None, None, None]

        If the pi vision cant see the target it will not send to network tables, meaning this will continue
        to return the most recently seen target position and the time it was seen
        """
        if self.entry.exists():
            self.data = self.entry.getDoubleArray([None, None, None])
            return self.data
        else:
            return [None, None, None]

    def is_ready(self) -> bool:
        if self.entry.exists() and self.data[2] != None:
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
