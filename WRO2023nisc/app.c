void bigship_nkc(){
    arm_take_ship();
    tslp_tsk(100*MSEC);
    //armで船掴む
    straight(15, -30);
    tslp_tsk(100*MSEC);
    turn(340, 0, 40);
    straight(40, 60);
    straight_on(30);
    while (true)
    {
        if(ev3_color_sensor_get_color(EV3_PORT_2) == COLOR_WHITE) break;
    }
    straight(13, 30);
    turn_color(-20, 20);
    linetrace_cm_pd(20, 0.7, 70, 35, false);
    straight(55, 80);
    //linetrace_cm_pd(40, 0.2, 45, 80, false);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30, false);
    straight(6.5, 30);
    turn(90, 24, -24);
    linetrace_cm_pd_SP(8.5, 20, false);
    linetrace_cm_pd_SP(13, 30, true);
    tslp_tsk(100*MSEC);
    straight(13, -30);
    //arm戻す
    arm_reset_A();
    tslp_tsk(400*MSEC);
    turn(192, 30, -30);
    straight(8, -20);
    arm_mode_change(LEFTDOWN);
    turn(15, -25, 25);
    arm_mode_change(RIGHTDOWN);
    ev3_motor_set_power(EV3_PORT_A, 40);
    tslp_tsk(400*MSEC);
    arm_mode_change(LEFTDOWN);
    tslp_tsk(200*MSEC);
    /*arm_mode_change(SET);
    tslp_tsk(200*MSEC);*/


    /*turn(180, 30, -30);
    straight(5, -15);
    arm_mode_change(LEFTDOWN);
    tslp_tsk(200*MSEC);
    turn(90, 70, 0);
    turn(90, 0, 70);
    straight(13.5, -30);
    arm_mode_change(RIGHTDOWN);
    ev3_motor_set_power(EV3_PORT_A, 40);
    tslp_tsk(400*MSEC);
    arm_mode_change(LEFTDOWN);
    tslp_tsk(100*MSEC);
    arm_mode_change(SET);
    tslp_tsk(200*MSEC);*/
}

void smallship_nkc(){
    linetrace_cm_pd_SP(10, 20, false);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 40, false);
    arm_set_A(115, false);
    linetrace_cm_pd_SP(16, 50, false);
    linetrace_cm_pd_SP(8, 10, true);
    /*linetrace_cm_pd_SP(20, 40, false);
    linetrace_cm_pd_SP(5.5, 20, true);*/
    //armで船掴む
    tslp_tsk(200*MSEC);
    arm_take_ship();
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 20, true);


    tslp_tsk(200*MSEC);
    turn(104, -30, 30);
    tslp_tsk(200*MSEC);
    straight(110, 70);
    tslp_tsk(200*MSEC);
    turn(82, -30, 30);
    arm_reset_A();
    tslp_tsk(400*MSEC);
    turn(186, 30, -30);
    straight(7, -15);
    //armおろしてオブジェクト下ろす
    arm_mode_change(RIGHTDOWN);
    tslp_tsk(100*MSEC);
    arm_mode_change(LEFTDOWN);
    tslp_tsk(100*MSEC);
    arm_mode_change(RIGHTDOWN);
    tslp_tsk(100*MSEC);
    straight_on(30);
    while (true)
    {
        if(ev3_color_sensor_get_color(EV3_PORT_2) == COLOR_WHITE && ev3_color_sensor_get_color(EV3_PORT_3) == COLOR_WHITE) break;
    }
   
    straight(9.5, 30);
    turn_color(30, -30);
}



//whitenkcの最初から貼ってある。最後の部分だけ変更あり。
    linetrace_cm_pd_SP(20, 30, false);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 60, false);
    linetrace_cm_pd_SP(6.2, 60, true);
    turn(90, 30, -30);
    arm_set_A(85, false);
    linetrace_cm_pd_SP(19, 30, false);
    linetrace_cm_pd_SP(13, 60, true);
    linetrace_cm_pd_SP(8.5, 20, true);
    tslp_tsk(100*MSEC);
    arm_set_A(40, true);
    tslp_tsk(100*MSEC);
    arm_take_obj();
    //armでオブジェクトとる

//ここまでがwhitenkc
   
    straight(10, -30);
    arm_reset_A();
    turn(90, -30, 30);
    straight(38, 60);
    straight_on(30);
    while (true){
        if(ev3_color_sensor_get_color(EV3_PORT_2) == COLOR_WHITE && ev3_color_sensor_get_color(EV3_PORT_3) == COLOR_WHITE) break;
    }
    while (true){
        if(ev3_color_sensor_get_color(EV3_PORT_2) == COLOR_BLACK && ev3_color_sensor_get_color(EV3_PORT_3) == COLOR_BLACK) break;
    }


    straight(32, 60);
    straight(8, 15);

//ここまでがbigprepare
