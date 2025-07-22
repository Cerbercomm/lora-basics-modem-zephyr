# My Cmake file
cmake_minimum_required(VERSION 3.20.0)

# We designate root dir to be a MODULE_EXT_ROOT (if/when this will accepted to the main zepher repo this wont be needed)
set(MODULE_EXT_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bermad_tag)

# Target and source files
FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
