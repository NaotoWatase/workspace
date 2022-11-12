int water_sp_count = 1;

void water_sp(int n) {
    if (location[n] == FIRE) {
        if (water_sp_count == 1)
            ev3_motor_rotate(EV3_PORT_D, 25 , -50, true);
            ev3_motor_rotate(EV3_PORT_D, 30 , -6, true);
            tslp_tsk(200*MSEC);
            ev3_motor_rotate(EV3_PORT_D, 55 , 20, true);
        }
        if (water_sp_count == 2)
            ev3_motor_rotate(EV3_PORT_D, 105 , -50, true);
            ev3_motor_rotate(EV3_PORT_D, 30 , -6, true);
            tslp_tsk(200*MSEC);
            ev3_motor_rotate(EV3_PORT_D, 135 , 20, true);
        }
    }
}

if (location[n] == FIRE) {
        if(water_count == 1) {
            ev3_motor_rotate(EV3_PORT_D, 20 , 50, true);
            ev3_motor_rotate(EV3_PORT_D, 30 , 6, true);
            tslp_tsk(200*MSEC);
            ev3_motor_rotate(EV3_PORT_D, 50 , -20, false);
        }
        if(water_count == 2) {
            ev3_motor_rotate(EV3_PORT_D, 100 , 50, true);
            ev3_motor_rotate(EV3_PORT_D, 30 , 6, true);
            tslp_tsk(200*MSEC);
            ev3_motor_rotate(EV3_PORT_D, 130 , -20, false);
        }
        water_count = water_count + 1;
    }