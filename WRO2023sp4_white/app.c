void sp_whiteobj_nkc(){
    arm_take_obj();
    linetrace_cm_pd_SP(20, 30, false);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 60, false);
    linetrace_cm_pd_SP(6.2, 60, true);
    turn(90, 30, -30);

    linetrace_cm_pd_SP(4, 20, false);
    linetrace_cm_pd_SP(15, 30, false);
    linetrace_cm_pd_SP(8.5, 10, true);
    tslp_tsk(200*MSEC);
    turn(180, 55, 0);
    tslp_tsk(100*MSEC);

    straight(2, 30);
    //obj_check(4, LEFT);

    if(両方とるやつ){
        straight(10.5, -30);
        turn(90, -30, 30);
        straight(8, -30);
        arm_reset_A();
        tslp_tsk(800*MSEC);
        arm_set_A(85, true);
        straight(11, 30);
        arm_set_A(40, true);
        tslp_tsk(100*MSEC);
        arm_take_obj();
        straight(38, -60);
        turn(90, -30, 30);
    }

    if(片方とるやつ){
        tslp_tsk(100*MSEC);
        straight(2, -30);
        turn(90, -30, 30);
        straight(8, -30);
        arm_reset_A();
        tslp_tsk(800*MSEC);
        arm_set_A(95, true);
        straight(11, 30);
        arm_set_A(30, true);
        tslp_tsk(100*MSEC);
        arm_take_obj();
        straight(11, -30);
        arm_reset_A();
        turn(90, -30, 30);
        straight(17, 30);
        turn(90, 30, -30);
        arm_set_A(95, true);
        straight(11, 30);
        arm_set_A(30, true);
        tslp_tsk(100*MSEC);
        arm_take_obj();
        turn(90, 30, -30);
        straight(9, 30);
        turn(90, 30, -30);
        linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30, false);
        linetrace_cm_pd_SP(6.5, 30, true);
        turn(90, 30, -30);
    }


    
    //armでオブジェクトとる
    /*straight(35.5, -60);
    turn(90, -30, 30);*/
}