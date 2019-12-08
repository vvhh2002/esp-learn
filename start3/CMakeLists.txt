cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_TOOLCHAIN_FILE /Users/Victor/esp/esp-idf/tools/cmake/toolchain-esp32.cmake)
set(IDF_TARGET esp32)
#include($ENV{IDF_PATH}/tools/cmake/project.cmake)

option(ESP_PLATFORM ON)
project(start3 C CXX)

add_executable(${CMAKE_PROJECT_NAME}.elf
        main/main.cpp
        main/hfp/hfp_hf_demo.c
        main/hfp/btstack_tlv_esp32.c
        main/hfp/btstack_stdin_esp32.c
        main/hfp/btstack_audio_esp32.c
        main/hfp/sco_demo_util.c
        )
include($ENV{IDF_PATH}/tools/cmake/idf_functions.cmake)
list(APPEND IDF_EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/components")
message("IDF_EXTRA_COMPONENT_DIRS:${IDF_EXTRA_COMPONENT_DIRS}")
set(IDF_BUILD_ARTIFACTS ON)
set(IDF_PROJECT_EXECUTABLE ${CMAKE_PROJECT_NAME}.elf)
set(IDF_BUILD_ARTIFACTS_DIR ${CMAKE_BINARY_DIR})
set(IDF_COMPONENTS freertos spi_flash bootloader esptool_py bt mqtt esp32 esp_http_server vfs mdns log btstack)
idf_import_components(components $ENV{IDF_PATH} esp-idf)
idf_link_components(${CMAKE_PROJECT_NAME}.elf "${components}")