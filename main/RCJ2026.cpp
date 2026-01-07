#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "ws2812.pio.h"
#include "hardware/adc.h"

#include "config.h"
#include "motor/motor.h"
#include "gyro/gyro.h"
#include "servo/servo.h"
#include "interface/interface.h"
#include "camera/camera.h"
typedef enum{
    LINE_NORMAL,
    LINE_CURVE,
    LINE_ZIGUZAG,
    LINE_T,
    LINE_CROSS,
    LINE_LOST,
    LINE_END,    
}linestate;
uint16_t photo_data[16];
int photo_th[16];

float r1, g1, b1, r2, g2, b2;
uint16_t tof = 0, loadcell = 0;

uint16_t left_sum_photo = 0;
uint16_t right_sum_photo = 0;
uint16_t all_sum_photo = 0;
uint16_t forward_photo = 0;

float line_position = 0.0f;
float last_line_position = 0.0f;
float diff = 0.0f;

float base_speed = 0.28f;
float p = 0.04f;
float d = 0.03f;

int loadcell_threshold = 1860;
uint16_t photo_threshold = 700;
uint16_t photo_forward = 2200;
float pitch_threshold = -10.0f;

int center_ms = 750;
int color_flag_l = 0;
int color_flag_r = 0;
float obstacle_angle = 0.0f;
bool saka = false;
float line_lose = 0.0f;
int line_skip = 0;

int weight[16] = {-7,-6,-5,-4,-3,-2,-1,0,0,1,2,3,4,5,6,7};
linestate detect_state(uint16_t photo_data[16], uint16_t *loadcell, uint16_t *tof, float *r1, float *g1, float *b1, float *r2, float *g2, float *b2){
    left_sum_photo = right_sum_photo = all_sum_photo = 0;
    line_position = 0.0f;
    color_flag_l = detect_color(*r1, *g1, *b1);
    color_flag_r = detect_color(*r2, *g2, *b2);
    for(int i=0;i<16;i++){
        if(i != 7)photo_th[i] = (photo_data[i] >= photo_threshold) ? 1 : 0;
        if(i == 7)photo_th[i] = (photo_data[i] >= photo_forward) ? 1 : 0;
        line_position += weight[i] * photo_th[i];
        if(i < 7) left_sum_photo += photo_th[i];
        if(i > 8) right_sum_photo += photo_th[i];
    }
    if(left_sum_photo + right_sum_photo > 0)line_position /= (left_sum_photo + right_sum_photo);
    else line_position = 0.0f;
    left_sum_photo += photo_th[8];
    right_sum_photo += photo_th[8];
    forward_photo = photo_th[7];
    all_sum_photo = left_sum_photo + right_sum_photo;
    if(all_sum_photo + forward_photo == 0)return LINE_LOST;
    if(all_sum_photo >= 10 &&(color_flag_l == 0 || color_flag_r == 0))return LINE_CROSS;
    if(all_sum_photo >= 10 && (color_flag_l + color_flag_r == 0)) return LINE_T;
    if(forward_photo == 1 && (left_sum_photo >=  7 && right_sum_photo <= 2))return LINE_T;
    if(forward_photo == 1 && (right_sum_photo >= 7 && left_sum_photo <= 2))return LINE_T;
    if(forward_photo == 0 && (left_sum_photo >= 6 && right_sum_photo <= 2))return LINE_ZIGUZAG;
    if(forward_photo == 0 && (right_sum_photo >= 6 && left_sum_photo <= 2))return LINE_ZIGUZAG;
    if(abs(left_sum_photo - right_sum_photo) >= 4)return LINE_CURVE;
    return LINE_NORMAL;
}
void calc_error(uint16_t photo_data[16]){
    int count = 0;
    for(int i=0;i<16;i++){
        if(photo_data[i] >= photo_threshold){
            line_position += weight[i];
            count++;
        }
    }
    if(count > 0) line_position /= count;
    else line_position = 0.0f;
    diff = line_position - last_line_position;
    last_line_position = line_position;
}
void linetrace(DualMotor &motor, uint16_t photo_data[16], uint16_t *loadcell, uint16_t *tof, float *r1, float *g1, float *b1, float *r2, float *g2, float *b2){
    linestate state = detect_state(photo_data, loadcell, tof, r1, g1, b1, r2, g2, b2);
    printf("| LEFT R:%.2f G:%.2f B:%.2f | RIGHT R:%.2f G:%.2f B:%.2f\n",*r1, *g1, *b1, *r2, *g2, *b2);
    bool edge_l = false;
    bool edge_r = false;
    calc_error(photo_data);
    switch(state){
        case LINE_NORMAL:
            red_led();
            motor.run(
                base_speed + p * line_position + d * diff,
                base_speed - p * line_position - d * diff
            );
            break;
        case LINE_CURVE:
            blue_led();
            motor.run(
                0.26f + 0.045 * line_position + 0.05 * diff,
                0.26f - 0.045 * line_position - 0.05 * diff
            );
            break;
        case LINE_ZIGUZAG:
            white_led();
            if(line_position>0){
                motor.stop(500);
                motor.run(0.30f, 0.30f);
                sleep_ms(center_ms);
                motor.turn(read_angle() + 90,forward_photo);
                motor.run(base_speed, base_speed);
                sleep_ms(700);
            }
            else{
                motor.stop(500);
                motor.run(0.30f, 0.30f);
                sleep_ms(center_ms);
                motor.turn(read_angle() - 90,forward_photo);
                motor.run(base_speed, base_speed);
                sleep_ms(700);
            }
            break;
        case LINE_T:
            buzzer();
    
            motor.run(base_speed, base_speed);
            sleep_ms(300);  
            break;
        case LINE_CROSS:
            green_led();
            motor.stop(500);
            motor.run(0.35f, 0.35f);
            sleep_ms(center_ms);
            motor.stop(200);
            if(color_flag_l == 0 && color_flag_r == 0){
                motor.obstacle_turn(read_angle() - 180);
            }
            else if(color_flag_l == 0){
                motor.obstacle_turn(read_angle() - 90);
            }
            else if(color_flag_r == 0){
                motor.obstacle_turn(read_angle() + 90);
            }
            motor.stop(300);
            motor.run(base_speed, base_speed);
            sleep_ms(700);
            break;
        case LINE_LOST:
            buzzer();
            motor.run(base_speed, base_speed);
            line_lose = true;
            sleep_ms(300);
            break;
        default:
            buzzer();
            break;
    }
}
int main(){
    stdio_init_all();
    sleep_ms(2000);
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(tx_pin1, GPIO_FUNC_UART);
    gpio_set_function(rx_pin1, GPIO_FUNC_UART);

    gpio_put(gyro_reset, 1);
    sleep_ms(100);
    gpio_put(gyro_reset, 0);
    sleep_ms(100);
    init_bno055();

    SERVO servo_arm(servo_3);
    SERVO servo_left(servo_2);
    SERVO servo_right(servo_1);
    SERVO servo_kago(servo_4);
    DualMotor motor(dc_left_1, dc_left_2, false,
                    dc_right_1, dc_right_2, false);

    ws2812_program_init(WS2812_PIN, 800000, IS_RGBW);
    led_on();
    buzzer();
    sleep_ms(500);
    while(1){
        if(line(photo_data, &loadcell, &tof,&r1, &g1, &b1, &r2, &g2, &b2)){
        if(line_skip < 5){
            line_skip++;
            motor.stop(10);  
            continue;        
        }
            red_led();
            printf("MID_PHOTO : %d\n",photo_data[7]);
            printf("LOAD_CELL : %d\n",loadcell);
            if(read_pitch() <= pitch_threshold){
                base_speed = 0.5f;
            }else if(read_pitch() >= -pitch_threshold){
                base_speed = 0.21;
            }
            else{
                base_speed = 0.30f;
            }
            linetrace(motor, photo_data, &loadcell, &tof, &r1, &g1, &b1, &r2, &g2, &b2);
        }
        else{
            yellow_led();
            motor.stop(200);
        }
    }
}