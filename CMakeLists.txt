# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(MotionLedHat C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(MotionLedHat main.cpp)

# Add any other source code
add_subdirectory(pico_i2c_slave)

pico_enable_stdio_uart(MotionLedHat 1)
pico_enable_stdio_usb(MotionLedHat 0)

# Add the standard library to the build
target_link_libraries(MotionLedHat pico_stdlib)

# Add any user requested libraries
target_link_libraries(MotionLedHat
        hardware_pwm
        i2c_slave
        )

pico_add_extra_outputs(MotionLedHat)

