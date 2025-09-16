// TB67H450_PWM.h
#ifndef TB67H450_PWM_H
#define TB67H450_PWM_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

class TB67H450{
private:
    uint pin_in1;
    uint pin_in2;
    bool direction;
    void setPWM(uint pin, float duty); 

public:
    TB67H450(uint in1, uint in2, bool forward);
    void run(float speed);   
    void stop(float time);                 
};

#endif
