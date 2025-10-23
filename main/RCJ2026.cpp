#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "ws2812.pio.h"

#include "config.h"
#include "motor/motor.h"
#include "VL53L0X/VL53L0X.h"
#include "gyro/gyro.h"
#include "servo/servo.h"
#include "interface/interface.h"
#include "camera/camera.h"
float angle = 0;

int main(){
    stdio_init_all();
    gpio_put(gyro_reset,1);
    sleep_ms(100);
    gpio_put(gyro_reset,0);
    sleep_ms(1000);
    init_bno055();
    ControlPacket packet;
    VL53L0X tof_left(tof_1,VL53L0X_DEFAULT_ADDRESS);
    VL53L0X tof_right(tof_2,VL53L0X_DEFAULT_ADDRESS);    
    DualMotor motor(dc_left_1,dc_left_2,true,dc_right_1,dc_right_2,true);
    SERVO servo_left(servo_1);
    SERVO servo_right(servo_2);
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(tx_pin1, GPIO_FUNC_UART);
    gpio_set_function(rx_pin1, GPIO_FUNC_UART);
    //ここまで設定
    led_on();
    ws2812_program_init(WS2812_PIN,800000,IS_RGBW);
    motor.run(0.8f,0.8f);
    sleep_ms(200);
    while(true) {
        motor.turn(0);
        blue_led();
        sleep_ms(2000);
        motor.turn(90);
        red_led();
        sleep_ms(2000);
        motor.turn(180);
        green_led();
        sleep_ms(2000);
        motor.turn(270);
        yellow_led();
        sleep_ms(2000);
        // if(angle>180)blue_led();
        // else red_led();
        //camera_line(&packet);
    }
}