//左から三つめ
    if(location_t[0] == 4) {
        linetrace_cm_pd_pro(15, 0, 13.5, 40, true);
        arm_D(DOWN);
        arm_A(SETNEW);
        tslp_tsk(100*MSEC);
        turn(10, 30, -30);
        tslp_tsk(100*MSEC);
        straight(8, 30);
        arm_A(CLOSE);
        straight(8, -30);
        turn(170, 30, -30);
        linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30, false);
        linetrace_cm_pd_SP(6.5, 20, true);
        
    }
//左から二つ目
    if(location_t[0] == 2) {
        linetrace_cm_pd_pro(15, 0, 13.5, 40, true);
        arm_D(DOWN);
        arm_A(SETNEW);
        tslp_tsk(100*MSEC);
        turn(10, -30, 30);
        tslp_tsk(100*MSEC);
        straight(8, 30);
        arm_A(CLOSE);
        straight(8, -30);
        turn(170, -30, 30);
        linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30, false);
        linetrace_cm_pd_SP(6.5, 20, true);
        
    }
//左から一つめ
    if(location_t[0] == 3) {
        linetrace_cm_pd_pro(12, 0, 14, 30, true);
        turn(90, -30, 30);
        straight(12, 20);
        turn(90, 30, -30);
        arm_D(DOWN);
        arm_A(SETNEW);
        straight(10, 20);
        arm_A(CLOSE);
        straight(10, -20);
        turn(90, 30, -30);
        straight(12, 20);
        turn(90, 30, -30);
        linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30, false);
        linetrace_cm_pd_SP(6.5, 20, true);
    }
//左から４つ目
    if(location_t[0] == 5) {
        linetrace_cm_pd_pro(12, 0, 14, 30, true);
        turn(90, 30, -30);
        straight(12, 20);
        turn(90, -30, 30);
        arm_D(DOWN);
        arm_A(SETNEW);
        straight(10, 20);
        arm_A(CLOSE);
        straight(10, -20);
        turn(90, -30, 30);
        straight(12, 20);
        turn(90, -30, 30);
        linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30, false);
        linetrace_cm_pd_SP(6.5, 20, true);
    }