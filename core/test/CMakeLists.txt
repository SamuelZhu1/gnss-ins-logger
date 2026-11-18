find_package(GTest REQUIRED)

add_executable(gins_tests
  sim.cpp
  test_alignment.cpp
  test_geo.cpp
  test_ins_filter.cpp
  test_log_format.cpp
  test_navigator.cpp
  test_time_base.cpp
  test_ubx.cpp)
target_link_libraries(gins_tests PRIVATE gins_core GTest::gtest_main)
target_compile_options(gins_tests PRIVATE -Wall -Wextra -Werror)

include(GoogleTest)
gtest_discover_tests(gins_tests)
