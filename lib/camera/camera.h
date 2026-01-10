#pragma once

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "../config.h"



#ifdef __cplusplus
extern "C" {
#endif
static void uart_read_u16_blocking(uint16_t *val);
bool line(uint16_t line[16], uint16_t *loadcell, uint16_t *tof,
          uint16_t *sum_l, uint16_t *sum_r,
          float *r1f, float *g1f, float *b1f,
          float *r2f, float *g2f, float *b2f);
static void uart_wait_header(void);
int detect_color(float r, float g, float b);
#ifdef __cplusplus
}
#endif