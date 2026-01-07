#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "motor.h"
#include "hardware/pwm.h"
#include "gyro.h"
#include "camera.h"


TB67H450::TB67H450(int in1, int in2,bool forward) {
    pin_in1 = in1;
    pin_in2 = in2;
    direction = forward;
    gpio_set_function(pin_in1, GPIO_FUNC_PWM);
    gpio_set_function(pin_in2, GPIO_FUNC_PWM);
    uint slice1 = pwm_gpio_to_slice_num(pin_in1);
    uint slice2 = pwm_gpio_to_slice_num(pin_in2);
    float clkdiv = 1.0f;
    uint32_t wrap = 24999;
    pwm_set_clkdiv(slice1,clkdiv);
    pwm_set_clkdiv(slice2,clkdiv);
    pwm_set_wrap(slice1, wrap);
    pwm_set_wrap(slice2, wrap);
    pwm_set_enabled(slice1, true);
    pwm_set_enabled(slice2, true);
    stop(0); 
}
void TB67H450::setPWM(int pin, float duty) {
    duty = fminf(fmaxf(duty, 0.0f), 1.0f); 
    uint slice = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice, channel, (uint16_t)(duty * 24999));
}

void TB67H450::run(float speed){
    if(!direction)speed *= -1;
    if(speed > 0.8f) speed = 0.8f;
    if(speed < -0.8f) speed = -0.8f;
    if(speed == 0.0f) speed = 0.4f;
    if(0 < speed && speed < 0.2f) speed = 0.2f;
    if(-0.2f < speed && speed < 0) speed = -0.2f;
    if(speed > 0.0f){
        setPWM(pin_in1, speed);
        setPWM(pin_in2, 0.0f); 
    } 
    else{
        setPWM(pin_in1, 0.0f);
        setPWM(pin_in2, fabs(speed));    
    }
}

void TB67H450::stop(float time) {
    setPWM(pin_in1, 1.0f);
    setPWM(pin_in2, 1.0f);
    sleep_ms(time);
}
DualMotor::DualMotor(int in1_l, int in2_l, bool forward_l, int in1_r, int in2_r, bool forward_r)
:motor_r(in1_r, in2_r, forward_r),
motor_l(in1_l, in2_l, forward_l) {
        pin1_l = in1_l;
        pin1_r = in1_r;
        pin2_l = in2_l;
        pin2_r = in2_r;
}
void DualMotor::setPWM(int pin, float duty) {
    duty = fminf(fmaxf(duty, 0.0f), 1.0f);
    uint slice = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice, channel, (uint16_t)(duty * 24999));
}
void DualMotor::run(float speed_l,float speed_r){
    motor_r.run(speed_r);
    motor_l.run(speed_l);
}

float normalize360(float angle) {
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}
void DualMotor::obstacle_turn(float target_angle){
    target_angle = normalize360(target_angle);  
        while (true) {
        float current_angle = normalize360(read_angle());  
        float diff = target_angle - current_angle;
        if (diff > 180.0f) diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;
        if (fabsf(diff) < 1.0f) { 
            stop(50);
           // printf("finish! T:%.2f C:%.2f D:%.2f\n", target_angle, current_angle, diff);
            break;
        }
        float speed = fminf(fmaxf(fabsf(diff) / 90.0f, 0.35f), 0.5f);

        if (diff > 0) {
            motor_r.run(-speed);
            motor_l.run(speed);
        } else {
            motor_r.run(speed);
            motor_l.run(-speed);
        }
    }
}
void DualMotor::turn(float target_angle, int photo_forward){
    target_angle = normalize360(target_angle);  
    uint16_t photo_data[16];
    int photo_th[16];

    float r1, g1, b1, r2, g2, b2;
    uint16_t tof = 0, loadcell = 0;
    while (true) {
        float current_angle = normalize360(read_angle());  
        float diff = target_angle - current_angle;
        if (diff > 180.0f) diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;
        if (fabsf(diff) < 20.0f && photo_forward == 1) { 
            stop(50);
           // printf("finish! T:%.2f C:%.2f D:%.2f\n", target_angle, current_angle, diff);
            break;
        }
        float speed = fminf(fmaxf(fabsf(diff) / 90.0f, 0.30f), 0.4f);

        if (diff > 0) {
            motor_r.run(-speed);
            motor_l.run(speed);
        } else {
            motor_r.run(speed);
            motor_l.run(-speed);
        }
    
     //   printf("T:%.2f C:%.2f D:%.2f\n", target_angle, current_angle, diff);
        line(photo_data, &loadcell, &tof,&r1, &g1, &b1, &r2, &g2, &b2);
        if(photo_data[7] >= 1600)photo_forward = 1;
        else photo_forward = 0;
        printf("%d",photo_data[7]);
        printf("%d\n",photo_forward);

        sleep_ms(20);
    }
}


void DualMotor::stop(float time){
    setPWM(pin1_l,1.0f);    
    setPWM(pin1_r,1.0f);  
    setPWM(pin2_l,1.0f);  
    setPWM(pin2_r,1.0f);  
    sleep_ms(time);
}