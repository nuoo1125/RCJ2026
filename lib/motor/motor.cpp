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
    pwm_set_wrap(slice1, wrap);
    pwm_set_wrap(slice2, wrap);
    pwm_set_enabled(slice1, true);
    pwm_set_enabled(slice2, true);
    stop(0); 
}

void TB67H450::setPWM(int pin, float duty) {
    uint slice = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice, channel, (uint16_t)(duty * 24999));
}

void TB67H450::run(float speed){
    if(!direction)speed *= -1;
    if(speed > 0){
        setPWM(pin_in1, speed);
        setPWM(pin_in2, 0.0f); 
    } 
    else{
        setPWM(pin_in1, 0.0f);
        setPWM(pin_in2, speed);    
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
    uint slice = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice, channel, (uint16_t)(duty * 24999));
}

void DualMotor::run(float speed){
    motor_r.run(speed);
    motor_l.run(speed);
}
void DualMotor::turn(int16_t target_angle){
    target_angle = (target_angle + 720) % 360;
    int16_t current_angle = read_angle();
    while(abs(target_angle-current_angle)>=0.5){
        motor_r.run(1.0f);
        motor_l.run(-1.0f);
        current_angle = read_angle();
    }
   printf("finish!");
}
void DualMotor::stop(float time){
    setPWM(pin1_l,1.0f);    
    setPWM(pin1_r,1.0f);  
    setPWM(pin2_l,1.0f);  
    setPWM(pin2_r,1.0f);  
    sleep_ms(time);
}