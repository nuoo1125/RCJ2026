#include "camera.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "../config.h"

#define HEADER 0xAA
#define PACKET_SIZE 11

static inline float bytes_to_float(uint8_t *data) {
    float f;
    memcpy(&f, data, sizeof(float));
    return f;
}
bool camera_line(ControlPacket *pkt){
    uint8_t buf[PACKET_SIZE];
    int b;
    do {
        b = uart_getc(UART_ID);
    } while (b != HEADER);

    buf[0] = (uint8_t)b;

    // 残りを読む
    for (int i = 1; i < PACKET_SIZE; i++) {
        buf[i] = uart_getc(UART_ID);
    }

    // チェックサム確認
    uint8_t checksum = buf[1];
    for (int i = 2; i < 10; i++) {
        checksum ^= buf[i];
    }
    if (checksum != buf[10]) {
        return false;
    }
    // 構造体に格納
    pkt->state = buf[1];
    pkt->left  = bytes_to_float(&buf[2]);
    pkt->right = bytes_to_float(&buf[6]);
    printf("state = %d, left = %.2f, right = %.2f\n",pkt->state, pkt->left, pkt->right);
    
    return true;
}
