from networktables import NetworkTablesInstance


class Vision:
    def __init__(self) -> None:
        self.ntinst = NetworkTablesInstance()
        self.ntinst.startClient("10.47.74.6")
        self.ntinst.setUpdateRate(1)

        self.vision_data_entry = self.ntinst.getEntry("/vision/vision_data")

    def get_vision_data(self) -> tuple:
        """Returns a tuple containing the distance (metres), angle (radians), and timestamp (time.monotonic)
        If it can't get info, it returns [-1.0, -1.0, -1.0]
        """
        self.vision_data_entry.getDoubleArray([-1.0, -1.0, -1.0])
