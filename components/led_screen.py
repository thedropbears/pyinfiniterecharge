import wpilib


class LEDScreen:
    led: wpilib.AddressableLED

    def __init__(self):
        self.led_length = 72
        self.led_bottom = wpilib.AddressableLED.LEDData(175, 0, 0)
        self.led_middle = wpilib.AddressableLED.LEDData(0, 175, 0)
        self.led_top = wpilib.AddressableLED.LEDData(0, 0, 175)

    def setup(self) -> None:
        self.led.setLength(self.led_length)
        self.led_rows = (
            [self.led_bottom] * int(self.led_length / 3)
            + [self.led_middle] * int(self.led_length / 3)
            + [self.led_top] * int(self.led_length / 3)
        )
        self.led.setData(self.led_rows)
        self.led.start()

    def set_bottom_row(self, r, g, b) -> None:
        self.led_bottom.setRGB(r, g, b)

    def set_middle_row(self, r, g, b) -> None:
        self.led_middle.setRGB(r, g, b)

    def set_top_row(self, r, g, b) -> None:
        self.led_top.setRGB(r, g, b)

    def execute(self) -> None:
        self.led.setData(self.led_rows)
