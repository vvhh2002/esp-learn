cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_TOOLCHAIN_FILE /Users/Victor/esp/esp-idf/tools/cmake/toolchain-esp32.cmake)
set(IDF_TARGET esp32)

option(ESP_PLATFORM ON)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS src/test_main src/components)
project(start4 C CXX)

