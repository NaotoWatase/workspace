//startがmooring areaだった場合。
    straight(8, 30);
    linetrace_cm_pd_SP(15, 10, false);
    linetrace_color_pd_SP(RIGHT, COLOR_BLACK, 30, true);


//スタートでオブジェクトを持っていて下ろす場合のアーム！！
//真ん中の線から18cmのところに真ん中寄りで落ちる。
    /*arm_mode_change(LEFTDOWN);
    tslp_tsk(100*MSEC);
    arm_mode_change(RIGHTDOWN);
    tslp_tsk(100*MSEC);
    arm_mode_change(SET);
    tslp_tsk(100*MSEC);*/

    linetrace_cm_pd_SP(6.5, 30, true);
    turn(90, 30, -30);
    linetrace_cm_pd_SP(12, 20, true);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 60, false);
    linetrace_cm_pd_SP(6, 60, false);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30, false);
    linetrace_cm_pd_SP(12, 30, true);
    turn(90, -30, 30);
    straight(13, 30);
    turn(90, 30, -30);
    straight_on(30);
    while (true)
    {
    if(ev3_color_sensor_get_color(EV3_PORT_2) == COLOR_WHITE && ev3_color_sensor_get_color(EV3_PORT_3) == COLOR_WHITE) break;
    }
    straight(11.5, 30);
    turn(90, 30, -30);
    straight(7, -30);
//!!!!!!!!!!startの6cmはいらない!!!!!!!!!!!!!





//bigshipの後に最初に落としたオブジェを持って、船のマークのところに置く。
    arm_reset_A();

    linetrace_cm_pd_SP(10, 30, true);
    linetrace_color_pd_SP(LEFT, COLOR_BLACK, 30, false);
    straight(20, 30);
    turn(80, 30, 0);
    turn(80, 0, 30);
    //
    arm_set_A(130, true);

    turn(180, 0, -30);
    tslp_tsk(200*MSEC);
    straight(17, 30);
    turn(90, 30, -30);
    straight(8, 30);
    arm_reset_A();


    straight(10, -30);
    arm_take_obj();
    turn(90, 30, -30);
    tslp_tsk(200*MSEC);

    straight_on(30);
    while (true){
        if(ev3_color_sensor_get_color(EV3_PORT_2) == COLOR_WHITE && ev3_color_sensor_get_color(EV3_PORT_3) == COLOR_WHITE) break;
    }
    while (true){
        if(ev3_color_sensor_get_color(EV3_PORT_2) == COLOR_BLACK && ev3_color_sensor_get_color(EV3_PORT_3) == COLOR_BLACK) break;
    }
    straight(6.5, 30);

    turn(90, 30, -30);
    tslp_tsk(200*MSEC);

    straight_on(30);
    tslp_tsk(2500*MSEC);

    straight(8, -20);

//コンテナをゆっくり、持ち上げずに保持したい場合。
//だいぶ緩めだから、5度あげてもいいい。

void arm_set_A(int arm_angle, bool_t stop){
    ev3_motor_stop(EV3_PORT_A, true);
    int power = 30;
    if (arm_angle == 30) power = 40;
    if (arm_angle == 130) power = 10;
    ev3_motor_rotate(EV3_PORT_A, arm_angle, power, stop);
    arm_set = arm_angle + arm_set;
}
