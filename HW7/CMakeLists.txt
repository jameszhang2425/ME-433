cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(nick_pico_examples C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(NICK_PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# Initialize the SDK
pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

add_executable(hw7)

target_sources(hw7 PRIVATE hw7.c)

pico_enable_stdio_usb(hw7 1)
pico_enable_stdio_uart(hw7 0)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(hw7 pico_stdlib hardware_i2c hardware_uart)

# create map/bin/hex file etc.
pico_add_extra_outputs(hw7)
