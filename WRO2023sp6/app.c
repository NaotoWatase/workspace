//bigshipの後、スタートエリア後ろのオブジェを持ちにいく。オブジェは持ち上げない
    linetrace_cm_pd_SP(10, 30, true);
    linetrace_color_pd_SP(LEFT, COLOR_BLACK, 30, false);
    linetrace_cm_pd_SP(6.5, 30, true);
    turn(90, -30, 30);
    linetrace_cm_pd_SP(12, 20, true);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 60, false);
    linetrace_cm_pd_SP(6, 60, false);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30, false);
    linetrace_cm_pd_SP(12, 30, true);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 20, false);
    straight(6.5, 30);
    turn(90, -30, 30);
    straight(20, 30);
    turn(90, 30, -30);
    straight(8, 30);
    turn(90, -30, 30);
    arm_reset_A();
    tslp_tsk(600*MSEC);
    straight(20, 30);
    arm_set_A(140, true);
    straight(16, -30);
    turn(90, -30, 30);
    straight(13, 30);
    turn(90, 30, -30);
    arm_reset_A();
    straight(20, 30);
    arm_set_A(140, true);
    straight(56, -30);
    turn(90, 30, -30);
    straight(7, -30);
    arm_reset_A();
    straight(10, -30);
    turn(90, -30, 30);
    straight_on(30);
        while (true)
        {
        if(ev3_color_sensor_get_color(EV3_PORT_2) == COLOR_WHITE && ev3_color_sensor_get_color(EV3_PORT_3) == COLOR_WHITE) break;
        }
    straight(10.5, 30);
    turn(90, -30, 30);
    linetrace_cm_pd_SP(100, 30, true);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30, true);
    turn(90, -30, 30);

    straight(30, 70);




//start_nkcの前、超音波を使ってスタートがどっちにいるか考える
if(ev3_ultrasonic_sensor_get_distance(PortSensorUltra4) > 15){
        turn(180, 30, -30);
        straight(8, 30);
        linetrace_cm_pd_SP(15, 10, false);
        linetrace_color_pd_SP(RIGHT, COLOR_BLACK, 30, false);
        linetrace_cm_pd_SP(6.5, 30, true);
        turn(90, 30, -30);
        linetrace_cm_pd_SP(12, 20, true);
        linetrace_color_pd_SP(BOTH, COLOR_BLACK, 60, false);
        linetrace_cm_pd_SP(6, 60, false);
        linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30, false);
        linetrace_cm_pd_SP(12, 30, true);
        turn(90, -30, 30);
        straight(12, 30);
        turn(90, 30, -30);
        straight_on(30);
        while (true)
        {
        if(ev3_color_sensor_get_color(EV3_PORT_2) == COLOR_WHITE && ev3_color_sensor_get_color(EV3_PORT_3) == COLOR_WHITE) break;
        }
        straight(10.5, 30);
        turn(90, 30, -30);
        straight(7, -30);

    }
    else {
        straight(6, 50);
    }