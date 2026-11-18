# Host build: the shared C++ core (filter, navigator, time base, UBX, log format)
# and its GoogleTest suites. The firmware builds the same sources under ESP-IDF
# (firmware/), and the Python tools build them into a pybind11 module (tools/).
cmake_minimum_required(VERSION 3.16)
project(gnss_ins_logger_host CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(GINS_BUILD_TESTS "Build the GoogleTest suites" ON)
option(GINS_USE_FLOAT "Build the filter in single precision, as on the ESP32" OFF)

add_subdirectory(core)

if(GINS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(core/test)
endif()
