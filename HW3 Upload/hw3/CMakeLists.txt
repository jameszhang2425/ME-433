add_executable(hw3)

target_sources(hw3 PRIVATE hw3.c)

pico_enable_stdio_usb(<PROJ_NAME> 1) pico_enable_stdio_uart(<PROJ_NAME> 0)


# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(hw3 pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(hw3)
