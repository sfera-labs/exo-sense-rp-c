pico_sdk_init()

add_subdirectory(lib/sensirion/sht4x)

add_executable(example
        example.c
        )

target_link_libraries(example pico_stdlib hardware_i2c exo_sense_rp sht4x)

# disable standard output on UART (used by RS-485)
pico_enable_stdio_uart(example 0)

# enable standard output on USB
pico_enable_stdio_usb(example 1)

pico_add_extra_outputs(example)
