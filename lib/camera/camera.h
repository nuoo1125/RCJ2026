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
typedef struct {
    uint8_t state;
    float left;
    float right;
} ControlPacket;
bool camera_line(ControlPacket *pkt);

#ifdef __cplusplus
}
#endif