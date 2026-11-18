idf_component_register(
  SRCS
    main.cpp
    icm42688.cpp
    gnss.cpp
    pps.cpp
    logger.cpp
    nav_task.cpp
    shell.cpp
    status.cpp
  INCLUDE_DIRS .
  REQUIRES gins_core driver esp_timer fatfs sdmmc vfs esp_app_format freertos)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
