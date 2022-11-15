int water_sp_count = 1;


void water_sp(int n) {
    if (location[n] == FIRE) {
        if (water_sp_count == 1) {
            ev3_motor_rotate(EV3_PORT_D, 55 , -30, true);
            ev3_motor_rotate(EV3_PORT_D, 30 , -7, true);
            tslp_tsk(200*MSEC);
            ev3_motor_rotate(EV3_PORT_D, 85 , 20, true);
        }
        if (water_sp_count == 2) {
            ev3_motor_rotate(EV3_PORT_D, 120 , -30, true);
            ev3_motor_rotate(EV3_PORT_D, 30 , -7, true);
            tslp_tsk(200*MSEC);
            ev3_motor_rotate(EV3_PORT_D, 150 , 20, true);
        }
    }
}

void water(int n) {
    if (location[n] == FIRE) {
        if(water_count == 1) {
            ev3_motor_rotate(EV3_PORT_D, 55 , 30, true);
            ev3_motor_rotate(EV3_PORT_D, 30 , 7, true);
            tslp_tsk(200*MSEC);
            ev3_motor_rotate(EV3_PORT_D, 85 , -20, false);
        }
        if(water_count == 2) {
            ev3_motor_rotate(EV3_PORT_D, 120 , 30, true);
            ev3_motor_rotate(EV3_PORT_D, 30 , 7, true);
            tslp_tsk(200*MSEC);
            ev3_motor_rotate(EV3_PORT_D, 150 , -20, false);
        }
        water_count = water_count + 1;
    }
}