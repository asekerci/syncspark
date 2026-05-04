idf_component_register(
    SRCS "src/mqtt_handler.c"
    INCLUDE_DIRS "include"
    REQUIRES mqtt drv8830 icm20948
    PRIV_REQUIRES syncspark_config nvs_flash
)
