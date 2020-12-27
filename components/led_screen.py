import wpilib

from components.logo import logo, logo_num_rows


class LEDScreen:
    led: wpilib.AddressableLED

    def __init__(self):
        self.led_length = 72
        self.led_row_length = int(self.led_length / 3)
        self.led_bottom_row_colour = wpilib.AddressableLED.LEDData(255, 0, 0)
        self.led_middle_row_colour = wpilib.AddressableLED.LEDData(0, 255, 0)
        self.led_top_row_colour = wpilib.AddressableLED.LEDData(0, 0, 255)
        self.led_raster = [
            [wpilib.AddressableLED.LEDData() for _ in range(self.led_row_length)]
            for _ in range(3)
        ]
        self.animating = False
        self.IMAGE_ROWS_BETWEEN_LED_ROWS = 5
        self.image = logo
        self.image_num_rows = logo_num_rows
        self.current_image_row = 0
        # Move the image after this number of ticks
        self.animation_period = 20
        self.current_animation_cycle = 0

    def setup(self) -> None:
        self.led.setLength(self.led_length)
        self.led_rows = (
            [self.led_bottom_row_colour] * self.led_row_length
            + [self.led_middle_row_colour] * self.led_row_length
            + [self.led_top_row_colour] * self.led_row_length
        )
        self.led.setData(self.led_rows)
        self.led.start()

    def set_bottom_row(self, r, g, b) -> None:
        self.led_bottom_row_colour.setRGB(r, g, b)

    def set_middle_row(self, r, g, b) -> None:
        self.led_middle_row_colour.setRGB(r, g, b)

    def set_top_row(self, r, g, b) -> None:
        self.led_top_row_colour.setRGB(r, g, b)

    def animate(self) -> None:
        self.current_image_row = 0
        self.load_row(self.current_image_row)
        self.animating = True
        self.led.setData(self.led_raster[0] + self.led_raster[1] + self.led_raster[2])

    def execute(self) -> None:
        if not self.animating:
            self.led.setData(self.led_rows)
        else:
            # step to next set of raster lines, if it's time
            self.current_animation_cycle = (
                self.current_animation_cycle + 1
            ) % self.animation_period
            if self.current_animation_cycle == 0:
                self.current_image_row = (
                    self.current_image_row + 1
                ) % self.image_num_rows
                self.load_row(self.current_image_row)
            self.led.setData(
                self.led_raster[0] + self.led_raster[1] + self.led_raster[2]
            )

    def load_row(self, image_row) -> None:
        second_image_row = (
            image_row + self.IMAGE_ROWS_BETWEEN_LED_ROWS
        ) % self.image_num_rows
        third_image_row = (
            second_image_row + self.IMAGE_ROWS_BETWEEN_LED_ROWS
        ) % self.image_num_rows
        for i in range(0, len(self.image[image_row])):
            self.led_raster[0][i].setRGB(
                self.image[image_row][i][0],
                self.image[image_row][i][1],
                self.image[image_row][i][2],
            )
            self.led_raster[1][i].setRGB(
                self.image[second_image_row][i][0],
                self.image[second_image_row][i][1],
                self.image[second_image_row][i][2],
            )
            self.led_raster[2][i].setRGB(
                self.image[third_image_row][i][0],
                self.image[third_image_row][i][1],
                self.image[third_image_row][i][2],
            )
