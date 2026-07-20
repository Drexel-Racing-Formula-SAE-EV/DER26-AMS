idf_component_register(
    SRCS
        "main.c"
    INCLUDE_DIRS "."
    REQUIRES plant_model CAN freertos
)