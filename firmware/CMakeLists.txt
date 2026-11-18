# ESP-IDF project for the ESP32-S3 logger. The filter, navigator, time base,
# UBX parser and log format come from ../core through components/gins_core.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(gnss_ins_logger)
