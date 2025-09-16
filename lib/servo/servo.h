#ifndef SERVO_H
#define SERVO_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

class SERVO {
private:
    uint pin_in1;
public:
    SERVO(int in1);
    void run(float angle);              
};

#endif