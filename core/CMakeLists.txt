find_path(EIGEN3_INCLUDE_DIR Eigen/Core
  HINTS $ENV{EIGEN3_INCLUDE_DIR}
  PATHS /usr/include/eigen3 /usr/local/include/eigen3 /opt/homebrew/include/eigen3)
if(NOT EIGEN3_INCLUDE_DIR)
  message(FATAL_ERROR "Eigen 3.4 not found: install libeigen3-dev or set EIGEN3_INCLUDE_DIR")
endif()

add_library(gins_core STATIC
  src/alignment.cpp
  src/geo.cpp
  src/ins_filter.cpp
  src/log_format.cpp
  src/navigator.cpp
  src/replay.cpp
  src/time_base.cpp
  src/ubx.cpp)
target_include_directories(gins_core PUBLIC include)
target_include_directories(gins_core SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
# Fixed-size matrices only: Eigen asserts if anything tries to allocate.
target_compile_definitions(gins_core PUBLIC EIGEN_NO_MALLOC)
if(GINS_USE_FLOAT)
  target_compile_definitions(gins_core PUBLIC GINS_USE_FLOAT)
endif()
target_compile_options(gins_core PRIVATE -Wall -Wextra -Wconversion -Werror)
set_target_properties(gins_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
