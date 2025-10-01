#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "ws2812.pio.h"

#include "config.h"
#include "motor/motor.h"
#include "VL53L0X/VL53L0X.h"
#include "gyro/gyro.h"
#include "servo/servo.h"
#include "interface/interface.h"

int main(){
    stdio_init_all();
    init_bno055();
    VL53L0X tof_left(tof_1,VL53L0X_DEFAULT_ADDRESS);
    VL53L0X tof_right(tof_2,VL53L0X_DEFAULT_ADDRESS);    
    DualMotor motor(dc_left_1,dc_left_2,true,dc_right_1,dc_right_2,false);
    SERVO servo_left(servo_1);
    SERVO servo_right(servo_2);
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(tx_pin1, GPIO_FUNC_UART);
    gpio_set_function(rx_pin1, GPIO_FUNC_UART);
    //ここまで設定
    led_on();
    ws2812_program_init(WS2812_PIN,800000,IS_RGBW);
    while (true) {
        red_led();
        sleep_ms(1000);
        blue_led();
        sleep_ms(1000);
    }
}