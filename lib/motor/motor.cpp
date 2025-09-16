#include "motor.h"
#include "hardware/pwm.h"
TB67H450::TB67H450(uint in1, uint in2,bool forward) {
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
    stop(2); 
}

void TB67H450::setPWM(uint pin, float duty) {
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
    setPWM(pin_in1, 0);
    setPWM(pin_in2, 0);
    sleep_ms(time);
}