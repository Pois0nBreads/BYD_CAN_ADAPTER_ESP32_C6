# idf_component_register(SRCS "hello_world_main.c"
#                        PRIV_REQUIRES spi_flash bt
#                        INCLUDE_DIRS "")

file(GLOB_RECURSE srcs "src/*.c")

idf_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS "./include"
    REQUIRES driver bt nvs_flash spi_flash
)
