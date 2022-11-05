
#include "ev3api.h"
#include "motor.h"
#include "app.h"
#include "stdlib.h"
#include <stdio.h>
#include"math.h"






void steering(int power, int cm, int steering) {
    int b_counts;
    int c_counts;
    cm = abs(cm);
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    power = -power;
    ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steering);
    while (true) {
        b_counts = abs(ev3_motor_get_counts(EV3_PORT_B));
        c_counts = abs(ev3_motor_get_counts(EV3_PORT_C));
        if (cm * ROBOT1CM < b_counts || cm * ROBOT1CM < c_counts) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}