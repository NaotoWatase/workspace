スタートの処理。最初に25cmに入る必要があるため

    now_arm_angle = 0;
    ev3_motor_set_power(EV3_PORT_D, 20);
    tslp_tsk(800*MSEC);
    //ev3_motor_stop(EV3_PORT_D, true);
    ev3_motor_reset_counts(EV3_PORT_D);



arm_take_shipの動き

    arm_mode_change_D_SPRZ(SETSHIP);
    arm_mode_change_A_SPRZ(SETOPEN);
    tslp_tsk(500*MSEC);
    straight(15, 30);
    arm_mode_change_D_SPRZ(UP);
    turn(90, -30, 30);
    straight(10,30);
    arm_mode_change_D_SPRZ(SETSHIP);
    tslp_tsk(500*MSEC);



床からオブジェをとって船の上に置く

    arm_mode_change_D_SPRZ(UP);
    tslp_tsk(500*MSEC);
    straight(10, 30);
    straight(2, -30);
    arm_mode_change_D_SPRZ(SETNORMAL);
    arm_mode_change_A_SPRZ(CLOSE);
    tslp_tsk(700*MSEC);
    arm_mode_change_D_SPRZ(UP);
    tslp_tsk(500*MSEC);
    straight(13, 30);
    arm_mode_change_D_SPRZ(SETSHIP);
    arm_mode_change_A_SPRZ(SETCLOSE);
    stopping();


船からオブジェをとって床に置く
    arm_mode_change_D_SPRZ(UP);
    tslp_tsk(500*MSEC);
    arm_mode_change_A_SPRZ(SETCLOSE);
    straight(13, 20);
    arm_mode_change_D_SPRZ(SETSHIP);
    arm_mode_change_A_SPRZ(CLOSE);
    tslp_tsk(500*MSEC);
    arm_mode_change_D_SPRZ(UP);
    tslp_tsk(500*MSEC);
    straight(13, -30);
    arm_mode_change_D_SPRZ(SETNORMAL);
    arm_mode_change_A_SPRZ(SETCLOSE);
    stopping();





サブアーム用

    void arm_D(armmode_new_t mode) {
    now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
    switch (mode) {
        case SETNORMAL:
            if(now_arm_angle >= -97)ev3_motor_set_power(EV3_PORT_D, -15);
            else ev3_motor_set_power(EV3_PORT_D, 15);
            break;
        case SETSHIP:
            if(now_arm_angle >= -60)ev3_motor_set_power(EV3_PORT_D, -15);
            else ev3_motor_set_power(EV3_PORT_D, 15);
            break;
        case DOWN:
            ev3_motor_set_power(EV3_PORT_D, -30);
            break;
        case UP:
            ev3_motor_set_power(EV3_PORT_D, 40);
            break;
        default:
            break;
    }
    while (true) {
        now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
        if(now_arm_angle <= -110 && mode == DOWN) break;
        //if(now_arm_angle >= -2 && mode == UP) break;
        if(mode == UP) break;
        if(now_arm_angle <= -96 && now_arm_angle >= -98 && mode == SETNORMAL) break;
        if(now_arm_angle <= -59 && now_arm_angle >= -61 && mode == SETSHIP) break;
    }
    if (mode == DOWN) {
        tslp_tsk(400*MSEC);
            ev3_motor_set_power(EV3_PORT_D, -6);
    }
    if (mode == UP) {
        tslp_tsk(400*MSEC);
    }
    else {
        ev3_motor_stop(EV3_PORT_D, true);
    }
    now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
}

void arm_A(armmode_new_t mode) {
    now_arm_angle_A = ev3_motor_get_counts(EV3_PORT_A);
    switch (mode) {
        case SETOPEN:
            if(now_arm_angle_A >= -60)ev3_motor_set_power(EV3_PORT_A, -20);
            else ev3_motor_set_power(EV3_PORT_A, 30);
            break;
        case SETCLOSE:
            if(now_arm_angle_A >= -155)ev3_motor_set_power(EV3_PORT_A, -20);
            else ev3_motor_set_power(EV3_PORT_A, 30);
            break;
        case OPEN:
            ev3_motor_set_power(EV3_PORT_A, 30);
            break;
        case CLOSE:
            ev3_motor_set_power(EV3_PORT_A, -40);
            break;
        default:
            break;
    }
    while (true) {
        now_arm_angle_A = ev3_motor_get_counts(EV3_PORT_A);
        if(now_arm_angle_A <= -90 && mode == CLOSE) break;
        //if(now_arm_angle >= -2 && mode == UP) break;
        if(mode == OPEN) break;
        if(now_arm_angle_A <= -59 && now_arm_angle >= -61 && mode == SETOPEN) break;
        if(now_arm_angle_A <= -154 && now_arm_angle >= -156 && mode == SETCLOSE) break;
    }
    if (mode == CLOSE) {
        tslp_tsk(600*MSEC);
            ev3_motor_set_power(EV3_PORT_A, -6);
    }
    if (mode == OPEN) {
        tslp_tsk(600*MSEC);
    }
    else {
        ev3_motor_stop(EV3_PORT_A, true);
    }
    now_arm_angle_A = ev3_motor_get_counts(EV3_PORT_A);
}

