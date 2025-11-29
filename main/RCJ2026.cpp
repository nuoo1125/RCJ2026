#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "ws2812.pio.h"
#include "hardware/adc.h"

#include "config.h"
#include "motor/motor.h"
#include "VL53L0X/VL53L0X.h"
#include "gyro/gyro.h"
#include "servo/servo.h"
#include "interface/interface.h"
#include "camera/camera.h"

const uint16_t LOAD_THRESHOLD = 2200;
const float PITCH_THRESHOLD = -10.0f;
float obstacle_angle = 0.0f;
int main(){
  stdio_init_all();
  gpio_put(gyro_reset,1);
  sleep_ms(100);
  gpio_put(gyro_reset,0);
  sleep_ms(100);
  init_bno055();
  ControlPacket packet;
  VL53L0X tof_left(tof_1,VL53L0X_DEFAULT_ADDRESS);
  VL53L0X tof_right(tof_2,VL53L0X_DEFAULT_ADDRESS);
  SERVO servo_left(servo_3);
  SERVO servo_right(servo_2);
  SERVO servo_kago(servo_1);
  SERVO servo_arm(servo_4);
  DualMotor motor(dc_left_1,dc_left_2,true,dc_right_1,dc_right_2,false);
  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function(tx_pin1, GPIO_FUNC_UART);
  gpio_set_function(rx_pin1, GPIO_FUNC_UART);

  adc_init();
  adc_gpio_init(26);
  adc_select_input(0);

  led_on();
  ws2812_program_init(WS2812_PIN,800000,IS_RGBW);
  sleep_ms(100);
  red_led();
  servo_arm.run(100);
  sleep_ms(200);
  bool obstacle = false;

  while (1) {
    if (!camera_line(&packet)) {
      sleep_ms(10);
      printf("camera error\n");
      continue;
    }
    blue_led();
    float angle = read_angle();
    uint16_t load_cell = adc_read();
    float pitch = read_pitch();
    if(load_cell > LOAD_THRESHOLD){
      obstacle_angle = read_angle();
      obstacle = true;
    }
    if (obstacle) {
      while (obstacle) {
        if (camera_line(&packet) && packet.state != 7) {
          obstacle = false;
          break;
        }
        blue_led();
        //motor.run(-0.4f, -0.4f);
        sleep_ms(800);
        //motor.stop(0);
        //motor.turn(read_angle() - 30.0f);
        while (1) {
          if (camera_line(&packet) && packet.state != 7) {
            obstacle = false;
            break;
          }
          //motor.run(0.3f, 0.6f);
          if (adc_read() > LOAD_THRESHOLD) {
            //motor.run(-0.4f, -0.4f);
            sleep_ms(800);
            //motor.turn(read_angle() - 30.0f);
          }
        }
      }
      //motor.turn(obstacle_angle);
      continue;
    }
    if (pitch < PITCH_THRESHOLD) {
      green_led();
      servo_arm.run(100);
      //motor.stop(1000);
      continue;
    }
    float befroe_left = packet.left/200.0f;
    float before_right = packet.right/200.0f;
     switch (packet.state) {

      case 2: {
        int count = 0;
        motor.stop(500);
        while (count < 3) {
          camera_line(&packet);
          if (packet.state == 2) count++;
          else{ 
          count = 0;
          break;
          }
          sleep_ms(10);
        }
        motor.run(0.4f,0.4f);
        sleep_ms(500);
        motor.turn(angle + 90);
        break;
      }

      case 3: {
        int count = 0;
        motor.stop(500);
        while (count < 3) {
          camera_line(&packet);
          if (packet.state == 3) count++;
          else{ 
          count = 0;
          break;
          }
          sleep_ms(10);
        }
        motor.run(0.4f,0.4f);
        sleep_ms(500);
        motor.turn(angle - 90);
        break;
      }

      case 4: {
        int count = 0;
        motor.stop(500);
        while (count < 3) {
          camera_line(&packet);
          if (packet.state == 4) count++;
          else{ 
          count = 0;
          break;
          }
          sleep_ms(10);
        }
        motor.run(0.4f,0.4f);
        sleep_ms(500);
        motor.turn(angle + 180);
        break;
      }

      case 1:
        motor.run(packet.left / 100.0f, packet.right / 100.0f);
        break;

      case 5:
        yellow_led();
        motor.stop(5000);
        break;

      case 6:
        green_led();
        motor.stop(5000);
        break;

      default:
        motor.stop(500);
        break;
    }
  }
  return 0;
}
