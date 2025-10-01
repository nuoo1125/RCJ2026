add_executable(RCJ2026 RCJ2026.cpp)

pico_set_program_name(RCJ2026 "RCJ2026")
pico_set_program_version(RCJ2026 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(RCJ2026 0)
pico_enable_stdio_usb(RCJ2026 1)

# Add the standard include files to the build
target_include_directories(RCJ2026 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)
target_include_directories(RCJ2026 PRIVATE ../lib)

target_link_libraries(RCJ2026  gyro VL53L0X servo interface motor pico_bootsel_via_double_reset)
target_link_libraries(RCJ2026 pico_stdlib hardware_pwm hardware_spi hardware_i2c hardware_pio hardware_clocks hardware_uart)

pico_add_extra_outputs(RCJ2026)