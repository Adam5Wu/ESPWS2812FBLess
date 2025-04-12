# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.10)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

get_filename_component(configName "${CMAKE_BINARY_DIR}" NAME)

# Add dependency libraries to the component search path.
set(EXTRA_COMPONENT_DIRS "..")
file(GLOB DEPS_DIRS LIST_DIRECTORIES true ".pio/libdeps/*/ZWUtils-IDF8266")
list(APPEND EXTRA_COMPONENT_DIRS ${DEPS_DIRS})

project(ESPWS2812FBLess_Example)
set(CMAKE_CXX_STANDARD 17)
