idf_component_register(
    SRCS
        "main.cpp"
        "sx1278.cpp"
        "gps_nmea.cpp"
        "compass_mpu.cpp"
        "buzzer.cpp"
        "encoder_input.cpp"
        "tracker.cpp"
        "display.cpp"

    INCLUDE_DIRS "."

    REQUIRES
        espressif__arduino-esp32
        Adafruit_BusIO
        Adafruit_GFX_Library
        Adafruit_ILI9341
)
