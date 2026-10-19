idf_component_register(
    SRCS "link_bench_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        radio
        rc_core
        esp_driver_uart
        esp_driver_gpio
        esp_timer
        esp_rom
        freertos
        log
)
