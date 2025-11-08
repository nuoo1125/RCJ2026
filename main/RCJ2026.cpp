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
  SERVO servo_left(servo_3);
  SERVO servo_right(servo_2);
  SERVO servo_kago(servo_1);
  SERVO servo_arm(servo_4);
  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function(tx_pin1, GPIO_FUNC_UART);
  gpio_set_function(rx_pin1, GPIO_FUNC_UART);
  //ここまで設定
  led_on();
  ws2812_program_init(WS2812_PIN,800000,IS_RGBW);
  sleep_ms(200);
  red_led();
  servo_kago.run(100);
  sleep_ms(500);
  while(1){
    if(camera_line(&packet)){
      blue_led();
      float angle = read_angle();
      if(read_pitch() > 10.0f){
        servo_kago.run(50);
      }
      else{
        servo_kago.run(100);
      }      
      if(packet.state == 1){
        motor.run(packet.left,packet.right);
      }
      else if(packet.state == 2){
        motor.turn(angle - 90);
      }
      else if(packet.state == 3){
        motor.turn(angle + 90);
      }
      else if(packet.state == 4){
        motor.turn(angle + 180);
      }
      else if(packet.state == 5){
        yellow_led();
        motor.stop(5000);
      }
      else if(packet.state == 6){
        green_led();
        motor.stop(5000);
      }
      sleep_ms(50);
    }
  }
}