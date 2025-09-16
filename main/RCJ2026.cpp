#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "ws2812.pio.h"

#include "config.h"
#include "motor/motor.h"
#include "VL53L0X/VL53L0X.h"
#include "gyro/gyro.h"
#include "servo/servo.h"
#include "interface/interface.h"

int main(){
    DualMotor motor(1,2,false,3,4,true);
    VL53L0X tof_l(i2c0);
    VL53L0X tof_r(i2c1);
}