set(COMPONENT_SRCS "main.c")
set(COMPONENT_ADD_INCLUDEDIRS "")
set(COMPONENT_REQUIRES freertos spi_flash bootloader esptool_py log nvs_flash btstack hfp_demo)
register_component()