file(GLOB SRC_LIST
    "*.c"
    "sensors/*.c"
    "drivers/*.c"
    "drivers/gps/*.c"
    "drivers/imu/*.c"
    "drivers/memory/*.c"
    "drivers/radio/*.c"
)

idf_component_register(
    SRCS ${SRC_LIST}
    INCLUDE_DIRS "." "sensors" "drivers" "drivers/gps" "drivers/imu" "drivers/memory" "drivers/radio"
    REQUIRES
        driver
        log
        freertos
    PRIV_REQUIRES
        spi_flash
)
