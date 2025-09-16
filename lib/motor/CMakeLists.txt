add_library(motor motor.c)

target_link_libraries(motor pico_stdlib hardware_pwm)