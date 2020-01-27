from networktables import NetworkTables


class Vision:
    def __init__(self) -> None:
        self.nt = NetworkTables
        self.table = self.nt.getTable("/vision")
        self.entry = self.table.getEntry("data")
        self.nt.setUpdateRate(1)

    def get_vision_data(self) -> tuple:
        """Returns a tuple containing the distance (metres), angle (radians), and timestamp (time.monotonic)
        If it can't get info, it returns [-1.0, -1.0, -1.0]
        """
        return self.entry.getDoubleArray([-1.0, -1.0, -1.0])
