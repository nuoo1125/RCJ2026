#include "gyro.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include "../config.h"

#define BNO055_ADDR ADDRESS
#define EULER_H_LSB 0x1A
#define EULER_H_MSB 0x1B

#define EULER_P_LSB 0x1E
#define EULER_P_MSB 0x1F

int16_t merge(uint8_t low, uint8_t high) {
    return (int16_t)((high << 8) | low);
}

void init_bno055() {
    uint8_t chip_id;

    i2c_init(gyro_i2c, 100000);
    gpio_set_function(gyro_sda, GPIO_FUNC_I2C);
    gpio_set_function(gyro_scl, GPIO_FUNC_I2C);
    gpio_pull_up(gyro_sda);
    gpio_pull_up(gyro_scl);

    gpio_set_dir(gyro_reset, GPIO_OUT);
    gpio_put(gyro_reset, 1);
    sleep_ms(100);
    gpio_put(gyro_reset, 0);
    sleep_ms(650);

    i2c_write_blocking(gyro_i2c, BNO055_ADDR, (uint8_t[]){0x00}, 1, true);
    i2c_read_blocking(gyro_i2c, BNO055_ADDR, &chip_id, 1, false);

    if (chip_id != 0xA0) {
        while (true) {
            printf("BNO055 not found...\n");
            sleep_ms(1000);
        }
    }

    uint8_t mode = 0x00;
    i2c_write_blocking(gyro_i2c, BNO055_ADDR, (uint8_t[]){0x3D, mode}, 2, false);
    sleep_ms(25);

    uint8_t unit_sel = 0x01;  
    i2c_write_blocking(gyro_i2c, BNO055_ADDR, (uint8_t[]){0x3B, unit_sel}, 2, false);
    sleep_ms(10);

    mode = 0x0C;  
    i2c_write_blocking(gyro_i2c, BNO055_ADDR, (uint8_t[]){0x3D, mode}, 2, false);
    sleep_ms(25);

    printf("BNO055 initialized.\n");
}
float read_angle() {
    uint8_t buf[2];

    i2c_write_blocking(gyro_i2c, BNO055_ADDR, (uint8_t[]){EULER_H_LSB}, 1, true);
    i2c_read_blocking(gyro_i2c, BNO055_ADDR, buf, 2, false);

    int16_t raw = merge(buf[0], buf[1]);
    return (float)raw / 16.0f;
}
float read_pitch() {
    uint8_t buf[2];
    i2c_write_blocking(gyro_i2c, BNO055_ADDR, (uint8_t[]){EULER_P_LSB}, 1, true);
    i2c_read_blocking(gyro_i2c, BNO055_ADDR, buf, 2, false);
    int16_t raw = (int16_t)((buf[1] << 8) | buf[0]);  // LSBがbuf[0], MSBがbuf[1]
    return (float)raw / 16.0f;  // -180 ~ +180° の範囲
}

