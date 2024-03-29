cmake_minimum_required(VERSION 3.5)
#include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(CMAKE_CXX_STANDARD 11)
project(idf_as_lib CXX C)

# The source file main.c contains app_main() definition
add_executable(${CMAKE_PROJECT_NAME}.elf custom.cpp)
#set(EXTRA_COMPONENT_DIRS "${EXTRA_COMPONENT_DIRS} ${CMAKE_SOURCE_DIR}/components")
# Provides idf_import_components() and idf_link_components()
include($ENV{IDF_PATH}/tools/cmake/idf_functions.cmake)
list(APPEND IDF_EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/components")
#include(${CMAKE_SOURCE_DIR}/components/customtest/CMakeLists.txt)

# Create artifacts used for flashing the project to target chip
set(IDF_BUILD_ARTIFACTS ON)
set(IDF_PROJECT_EXECUTABLE ${CMAKE_PROJECT_NAME}.elf)
set(IDF_BUILD_ARTIFACTS_DIR ${CMAKE_BINARY_DIR})


# Trim down components included in the build. Although freertos and spi_flash are the ones needed by the application
# itself, the bootloader and esptool_py components are also needed in order to create the artifacts to be used
# for flashing to the target chip
set(IDF_COMPONENTS freertos spi_flash bootloader esptool_py bt asio fatfs json mqtt esp_http_server vfs mdns log jsmn customtest)

#set(IDF_COMPONENT_DIRS "${IDF_COMPONENT_DIRS} ${CMAKE_SOURCE_DIR}/components/customtest")

# Wraps add_subdirectory() to create library targets for components, and then return them using the specified variable
idf_import_components(components $ENV{IDF_PATH} esp-idf)
message("LINK COMMPNENTS:${components}")

# Wraps target_link_libraries() to link processed components by idf_import_components to target
idf_link_components(${CMAKE_PROJECT_NAME}.elf "${components}")