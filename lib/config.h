#pragma once
//基本的に左から一番二番にしてある
//tx/rxはマイコンから見て
//gyro系
#define gyro_reset 14
#define gyro_i2c i2c0 
#define gyro_sda 16
#define gyro_scl 17
#define ADDRESS 0x28
#define EULER_REGISTER 0x1A
//dc系
#define dc_left_1 8
#define dc_left_2 9
#define dc_right_1 6
#define dc_right_2 7
//tof系
#define VL53L0X_DEFAULT_ADDRESS 0x29
#define tof_1 i2c0
#define tof_2 i2c1
#define sda_tof1 16
#define scl_tof1 17
#define sda_tof2 10
#define scl_tof2 11
#define xshut2 3
#define xshut1 2
//servo系
#define servo_1 20  
#define servo_2 21 //わんちゃん干渉する
#define servo_3 22 
#define servo_4 27 
//debagu系
#define buzzer_pin 15
#define led 25
#define WS2812_PIN 1
//センサ系
#define UART_ID uart0
#define BAUD_RATE 115200
#define tx_pin1 12 
#define rx_pin1 13 
#define tx_pin2 4
#define rx_pin2 5