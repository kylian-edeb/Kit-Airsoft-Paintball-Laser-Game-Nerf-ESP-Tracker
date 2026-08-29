idf_component_register(
    SRCS "Adafruit_ILI9341.cpp"
    INCLUDE_DIRS "."
    REQUIRES espressif__arduino-esp32 Adafruit_GFX_Library Adafruit_BusIO
)
