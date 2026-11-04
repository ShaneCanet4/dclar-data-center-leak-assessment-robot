"""Text status display for the Adafruit ESP32-S3 TFT Feather.

Uses the same st7789py, font, pins, and SPI setup as the board's known-working
display test. Copy st7789py.py and vga1_16x32.py to the ESP32 as well.
"""

from machine import Pin, SPI
import st7789py as st7789
import vga1_16x32 as font


class BoardDisplay:
    def __init__(self):
        # The board requires GPIO21 high to power the TFT connector.
        self.power = Pin(21, Pin.OUT)
        self.power.value(1)

        self.backlight = Pin(45, Pin.OUT)
        self.backlight.value(1)

        # Match the known-working board setup: SCK=GPIO36, MOSI=GPIO35.
        self.spi = SPI(1, baudrate=40_000_000, sck=Pin(36), mosi=Pin(35))
        self.tft = st7789.ST7789(
            self.spi,
            135,
            240,
            reset=Pin(40, Pin.OUT),
            cs=Pin(7, Pin.OUT),
            dc=Pin(39, Pin.OUT),
            backlight=self.backlight,
            rotation=1,
        )
        self.show(["Starting..."])

    def show(self, lines):
        """Draw a header and up to three wrapped status rows on the TFT."""
        self.tft.fill(st7789.BLACK)
        self.tft.text(font, "DCLAR ESP-NOW", 8, 0, st7789.CYAN, st7789.BLACK)

        # This VGA font is 16 pixels wide, so 14 characters fit in 224 pixels.
        display_lines = []
        for line in lines:
            text = str(line).replace("\n", " ")
            for index in range(0, len(text), 14):
                display_lines.append(text[index:index + 14])
        for index, line in enumerate(display_lines[:3]):
            self.tft.text(
                font, line, 8, 32 + index * 32, st7789.GREEN, st7789.BLACK
            )
