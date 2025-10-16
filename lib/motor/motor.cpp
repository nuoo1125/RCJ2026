#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "motor.h"
#include "hardware/pwm.h"
#include "gyro.h"
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
    if(speed > 1.0f) speed = 1.0f;
    if(speed < -1.0f) speed = -1.0f;
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
void DualMotor::turn(int16_t target_angle){
    target_angle = (target_angle % 360 + 360) % 360;

    int16_t current_angle = read_angle();
    current_angle = (current_angle % 360 + 360) % 360;

    const int16_t stop_threshold = 1;   
    const int16_t restart_threshold = 5; 
    bool turning = true;
    while (true) {
        int diff = (int)target_angle - (int)current_angle;
        diff = (diff + 540) % 360 - 180; // -180..179
        if (turning) {
            // 目標に到達したら停止
            if (abs(diff) <= stop_threshold) {
                stop(50);
                turning = false;
            } else {
                float speed = fminf(fmaxf(fabsf((float)diff) / 90.0f, 0.25f), 1.0f);
                if (diff > 0) {
                    motor_r.run( speed);
                    motor_l.run(-speed);
                } else {
                    motor_r.run(-speed);
                    motor_l.run( speed);
                }
            }
        } else {
            if (abs(diff) > restart_threshold) {
                turning = true;
            }
        }
        sleep_ms(10);
        current_angle = read_angle();
        current_angle = (current_angle % 360 + 360) % 360;

        if (!turning && abs(diff) <= stop_threshold) {
            break;
        }
    }
    printf("finish!\n");
}

void DualMotor::stop(float time){
    setPWM(pin1_l,1.0f);    
    setPWM(pin1_r,1.0f);  
    setPWM(pin2_l,1.0f);  
    setPWM(pin2_r,1.0f);  
    sleep_ms(time);
}