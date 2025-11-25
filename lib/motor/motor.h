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
        void setPWM(int pin, float duty); 
    public:
        TB67H450(int in1, int in2, bool forward);
        void run(float speed);   
        void stop(float time);                 
};
class DualMotor{
    private:
        TB67H450 motor_l;
        TB67H450 motor_r;
        int pin1_l;
        int pin1_r;
        int pin2_l;
        int pin2_r;
        void setPWM(int pin, float duty); 
    public:
        float normalize_angle(float angle);
        DualMotor(int in1_l,int in2_l,bool forward_l,int in1_r,int in2_r,bool forward_r);
        void run(float speed_l,float spped_r);
        void turn(float target_angle);
        // void turnRight();
        void stop(float time);
};
#endif