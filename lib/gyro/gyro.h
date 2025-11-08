#pragma once

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "../config.h"



#ifdef __cplusplus
extern "C" {
#endif
int16_t merge(uint8_t low, uint8_t high);
void init_bno055();
float read_angle();
float read_pitch();
#ifdef __cplusplus
}
#endif