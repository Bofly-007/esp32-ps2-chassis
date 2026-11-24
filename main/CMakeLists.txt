idf_component_register(
    SRCS "main.c" "bts7960.c" "motor_ramp.c" "ps2_bus.c" "ps2_control.c"
    INCLUDE_DIRS "."
    REQUIRES freertos log esp_system esp_timer esp_rom console esp_driver_gpio esp_driver_ledc
)
