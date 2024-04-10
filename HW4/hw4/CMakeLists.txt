add_executable(hw4
        hw4
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(hw3 pico_stdlib hardware_adc)

# create map/bin/hex file etc.
pico_add_extra_outputs(hw3)
