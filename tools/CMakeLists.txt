# Python extension: the same C++ core the firmware runs, built in double precision.
cmake_minimum_required(VERSION 3.16)
project(gins_py CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(pybind11 CONFIG REQUIRED)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../core ${CMAKE_BINARY_DIR}/core)

pybind11_add_module(_core src/gins_py.cpp)
target_link_libraries(_core PRIVATE gins_core)
install(TARGETS _core DESTINATION gins)
