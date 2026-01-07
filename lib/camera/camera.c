#include "camera.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "../config.h"
#include <math.h>

#define PACKET_SIZE 62  // floatを6個追加したので拡張

static void uart_wait_header(void)
{
    uint8_t c;

    while (true) {
        while (!uart_is_readable(UART_ID));
        c = uart_getc(UART_ID);

        if (c == 0xAA) {
            while (!uart_is_readable(UART_ID));
            c = uart_getc(UART_ID);

            if (c == 0x55) {
                return;
            }
        }
    }
}
bool line(uint16_t line[16], uint16_t *loadcell, uint16_t *tof,
          float *r1f, float *g1f, float *b1f,
          float *r2f, float *g2f, float *b2f)
{
    uart_wait_header();

    uint8_t buf[PACKET_SIZE-2];
    uart_read_blocking(UART_ID, buf, sizeof(buf));

    int idx = 0;
    for(int i=0;i<16;i++)
        line[i] = buf[idx++] | (buf[idx++]<<8);

    *loadcell = buf[idx++] | (buf[idx++]<<8);
    *tof      = buf[idx++] | (buf[idx++]<<8);

    memcpy(r1f, &buf[idx], sizeof(float)); idx += 4;
    memcpy(g1f, &buf[idx], sizeof(float)); idx += 4;
    memcpy(b1f, &buf[idx], sizeof(float)); idx += 4;
    memcpy(r2f, &buf[idx], sizeof(float)); idx += 4;
    memcpy(g2f, &buf[idx], sizeof(float)); idx += 4;
    memcpy(b2f, &buf[idx], sizeof(float)); idx += 4;

    return true;
}

int detect_color(float r, float g, float b){

    if(g > 0.0f && g < 0.40f && r < 0.35f && b < 0.20f){
        return 1; // black
    }
    else if(g > 0.30f && b < 0.15f){
        if(fabsf(r-g)<0.10f){
            return 1;
        }
        else return 0;
    }
    else {
        return 2; // white
    }
}
