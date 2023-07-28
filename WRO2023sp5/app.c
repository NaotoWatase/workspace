void obj_know(int num){
    switch(obj){
        case 2:
            location[num] = 2;
            break;
        case 3:
            location[num] = 3;
            break;
        case 4:
        case 7:
            location[num] = 4;
            break;
        case 0:
        case 1:
        case 5:
        case 6:
            if(h < 150) location[num] = 3;
            else location[num] = 2;
            break;
        default:
            break;
    } 
    if(num <= 1)location_t[num] = location[num];
    else location_f[num - 2] = location[num];
    fprintf(bt, "RESULT = T=%d,%d F=%d,%d,%d,%d\r\n-----------------\r\n", location_t[0], location_t[1], location_f[0], location_f[1], location_f[2], location_f[3]);
    file=fopen(logfilename,"a");//ファイルをオープン(名前の指定)
    fprintf(file, "RESULT = T=%d,%d F=%d,%d,%d,%d\r\n-----------------\r\n", location_t[0], location_t[1], location_f[0], location_f[1], location_f[2], location_f[3]);
    fclose(file);
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

    straight(17, -40);
    turn(90, -20, 20);
    linetrace_cm_pd(15, 0.6, 55, 28, false);
    //straight(15, 15);
    //linetrace_cm_pd(50, 0.15, 50, 15, false);
    //linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30, false);
    /*linetrace_cm_pd(20, 0.6, 55, 15, false);
    linetrace_cm_pd(50, 0.15, 50, 25, false);
    linetrace_cm_pd(20, 0.6, 55, 10, true);*/

    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30, false);
    linetrace_cm_pd_SP(9, 20, true);
    turn(90, 20, -20);
    straight(21.5, 30);
    tslp_tsk(100*MSEC);
    turn(60, -20, 20);
    turn(60, 20, -20);
    straight(21.5, -30);
    turn(90, -30, 30);
    linetrace_cm_pd_SP(10, 20, false);
    linetrace_cm_pd_SP(28, 40, true);


    tslp_tsk(100*MSEC);
    turn(194, 0, 50);
    //arm開く

    arm_reset_A();
    tslp_tsk(300*MSEC);
    turn(187, 30, -30);
    straight(6, -15);
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
    turn(90, 30, -30);
}