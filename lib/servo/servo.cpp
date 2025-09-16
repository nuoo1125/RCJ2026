#include "servo.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
SERVO::SERVO(int in1) {
    pin_in1 = in1;
    gpio_set_function(pin_in1, GPIO_FUNC_PWM);
    uint slice1 = pwm_gpio_to_slice_num(pin_in1);
    pwm_set_wrap(slice1, 25000);
    pwm_set_clkdiv(slice1,64.0f);

    pwm_set_enabled(slice1, true);
}
void SERVO::run(float angle){
    float duty_cycle = 0.5 + (angle * 2.0 / 180.0); 
    uint16_t level = (uint16_t)(duty_cycle * 25000.0 / 20.0);
    uint slice1 = pwm_gpio_to_slice_num(pin_in1); 
    pwm_set_chan_level(slice1, pwm_gpio_to_channel(pin_in1), level);
}

