#include "servo.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"

SERVO::SERVO(int in1) {
    pin_in1 = in1;
    gpio_set_function(pin_in1, GPIO_FUNC_PWM);
    uint slice1 = pwm_gpio_to_slice_num(pin_in1);  
    pwm_set_wrap(slice1, 24999);
    pwm_set_clkdiv(slice1,100.0f);
    pwm_set_enabled(slice1, true);
}

void SERVO::run(float angle){
    float duty_cycle = 0.5f + (angle * 1.9f / 180.0f); 
    uint16_t level = (uint16_t)(duty_cycle * 25000.0f/ 20.0f);
    uint slice1 = pwm_gpio_to_slice_num(pin_in1); 
    pwm_set_chan_level(slice1, pwm_gpio_to_channel(pin_in1), level);
}
