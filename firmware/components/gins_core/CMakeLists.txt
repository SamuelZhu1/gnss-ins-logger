# Builds the shared core (../../../core) for the ESP32-S3 in single precision.
set(core ${CMAKE_CURRENT_LIST_DIR}/../../../core)

idf_component_register(
  SRCS
    ${core}/src/alignment.cpp
    ${core}/src/geo.cpp
    ${core}/src/ins_filter.cpp
    ${core}/src/log_format.cpp
    ${core}/src/navigator.cpp
    ${core}/src/time_base.cpp
    ${core}/src/ubx.cpp
  INCLUDE_DIRS ${core}/include)

# Eigen is header-only. CI installs libeigen3-dev in the IDF container; on other
# machines point EIGEN3_INCLUDE_DIR at an Eigen 3.4 checkout.
if(DEFINED ENV{EIGEN3_INCLUDE_DIR})
  set(eigen_dir $ENV{EIGEN3_INCLUDE_DIR})
else()
  set(eigen_dir /usr/include/eigen3)
endif()
if(NOT EXISTS ${eigen_dir}/Eigen/Core)
  message(FATAL_ERROR "Eigen not found at ${eigen_dir}: set EIGEN3_INCLUDE_DIR")
endif()
target_include_directories(${COMPONENT_LIB} SYSTEM PUBLIC ${eigen_dir})
target_compile_definitions(${COMPONENT_LIB} PUBLIC GINS_USE_FLOAT EIGEN_NO_MALLOC EIGEN_NO_DEBUG)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
