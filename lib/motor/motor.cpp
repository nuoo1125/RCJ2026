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
    if(0 <= speed && speed < 0.22f) speed = 0.22f;
    if(-0.22f < speed && speed < 0) speed = -0.22f;
    if(speed >= 0.0f){
        setPWM(pin_in1, speed);
        setPWM(pin_in2, 0.0f); 
    } 
    else{
        setPWM(pin_in1, 0.0f);
        setPWM(pin_in2, fabs(speed));    
    }
}

void TB67H450::stop(float time) {
    setPWM(pin_in1, 0.0f);
    setPWM(pin_in2, 0.0f);
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
void DualMotor::turn(float target_angle, int photo_forward)
{
    uint16_t sum_l = 0, sum_r = 0;
    uint16_t photo_data[16];
    int photo_th[16];

    float r1, g1, b1, r2, g2, b2;
    uint16_t tof = 0, loadcell = 0;

    /* =========================
     * ① 中央センサが黒になるまで旋回
     * ========================= */
        float current_angle = normalize360(read_angle());
        float diff = target_angle - current_angle;
        float speed = 0.42f;
        if(diff > 0){
            motor_r.run(-speed);
            motor_l.run(speed);
        }
        else{
            motor_r.run(speed);
            motor_l.run(-speed);
        }
        sleep_ms(1000);
            setPWM(pin1_l,1.0f);    
            setPWM(pin1_r,1.0f);  
            setPWM(pin2_l,1.0f);  
            setPWM(pin2_r,1.0f);  
    while (true) {
        current_angle = normalize360(read_angle());
        diff = target_angle - current_angle;
        line(photo_data, &loadcell, &tof,
             &sum_l, &sum_r,
             &r1, &g1, &b1, &r2, &g2, &b2);
        for (int i = 0; i < 16; i++) {
            photo_th[i] = (photo_data[i] >= 1800) ? 1 : 0;
        }
        if (photo_th[7] >= 1) {
            stop(50);
            break;
        }

        if(diff > 0){
            motor_r.run(-speed);
            motor_l.run(speed);
        }
        else{
            motor_r.run(speed);
            motor_l.run(-speed);
        }

        sleep_ms(10);
    }

    /* =========================
     * ② 首振りでライン復帰（gyroなし）
     * ========================= */
    int dir = 1;                 // 1:右 → -1:左
    absolute_time_t t0 = get_absolute_time();

    while (true) {
        line(photo_data, &loadcell, &tof,
             &sum_l, &sum_r,
             &r1, &g1, &b1, &r2, &g2, &b2);

        for (int i = 0; i < 16; i++) {
            photo_th[i] = (photo_data[i] >= 1300) ? 1 : 0;
        }
        if (photo_th[7] >= 1) {
            stop(50);
            return;
        }
        float speed = 0.35f;
        motor_r.run(-dir * speed);
        motor_l.run(dir * speed);

        if (absolute_time_diff_us(t0, get_absolute_time()) > 250000) {
            dir = -dir;
            t0 = get_absolute_time();
        }

        sleep_ms(10);
    }
}



void DualMotor::stop(float time){
    setPWM(pin1_l,0.0f);    
    setPWM(pin1_r,0.0f);  
    setPWM(pin2_l,0.0f);  
    setPWM(pin2_r,0.0f);  
    sleep_ms(time);
}
