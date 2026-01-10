#include "camera.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "../config.h"
#include <math.h>
#include <string.h>

#define PACKET_SIZE 56   // ★sum_l, sum_r 追加

/* ================= ヘッダ待ち ================= */
static void uart_wait_header(void)
{
    uint8_t c;

    while (true) {
        while (!uart_is_readable(UART_ID));
        c = uart_getc(UART_ID);

        if (c == 0xAA) {
            while (!uart_is_readable(UART_ID));
            if (uart_getc(UART_ID) == 0x55) {
                return;
            }
        }
    }
}

/* ================= UART受信 ================= */
bool line(uint16_t line[16], uint16_t *loadcell, uint16_t *tof,
          uint16_t *sum_l, uint16_t *sum_r,
          float *r1f, float *g1f, float *b1f,
          float *r2f, float *g2f, float *b2f)
{
    uart_wait_header();

    uint8_t buf[PACKET_SIZE - 2];   // header除外
    uart_read_blocking(UART_ID, buf, sizeof(buf));

    int idx = 0;

    // ライン
    for(int i=0;i<16;i++){
        line[i] = buf[idx++] | (buf[idx++] << 8);
    }

    // loadcell / tof
    *loadcell = buf[idx++] | (buf[idx++] << 8);
    *tof      = buf[idx++] | (buf[idx++] << 8);

    // sum
    *sum_l = buf[idx++] | (buf[idx++] << 8);
    *sum_r = buf[idx++] | (buf[idx++] << 8);

    // RGB（0〜1000 → float）
    uint16_t r1n = buf[idx++] | (buf[idx++] << 8);
    uint16_t g1n = buf[idx++] | (buf[idx++] << 8);
    uint16_t b1n = buf[idx++] | (buf[idx++] << 8);

    uint16_t r2n = buf[idx++] | (buf[idx++] << 8);
    uint16_t g2n = buf[idx++] | (buf[idx++] << 8);
    uint16_t b2n = buf[idx++] | (buf[idx++] << 8);

    *r1f = r1n / 1000.0f;
    *g1f = g1n / 1000.0f;
    *b1f = b1n / 1000.0f;

    *r2f = r2n / 1000.0f;
    *g2f = g2n / 1000.0f;
    *b2f = b2n / 1000.0f;

    return true;
}

/* ================= 色判定 ================= */
int detect_color(float r, float g, float b)
{
    if(g > 0.0f && g < 0.40f && r < 0.35f && b < 0.20f){
        return 1; // black
    }
    else if(g > 0.30f && b < 0.15f){
        if(fabsf(r-g) < 0.10f){
            return 1;
        }
        else return 0;
    }
    else {
        return 2; // white
    }
}
