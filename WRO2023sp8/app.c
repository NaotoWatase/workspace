//objprepareのはじめに、bigshipにfuelを置きにいく
    linetrace_cm_pd_SP(13.5, 20, false);
    turn(90, 15, -15);
    straight(10.5, -10);
    tslp_tsk(400*MSEC);
    straight(10.5, 10);
    turn(90, -20, 20);



//goalの後で行う　壁の代わりにオブジェクトが置かれ、インフォメーションの組み合わせによって正しい方を取ってゴール　ゴールは完全には入らない。
    arm_reset_A();
    linetrace_cm_pd_SP(5, 30, true);
    linetrace_color_pd_SP(LEFT, COLOR_BLACK, 30, false);
    straight(6.5, 30);
    turn(90, -30, 30);
    linetrace_cm_pd_SP(38, 30, true);
    if(location_t[0] == location_t[1]){
        turn(90, 30, -30);
        straight(11, 30);
        arm_set_A(135, true);
        tslp_tsk(200*MSEC);
        straight(11, -30);
        turn(90, 30, -30);
    }
    else{
        turn(90, -30, 30);
        straight(11, 30);
        arm_set_A(135, true);
        tslp_tsk(200*MSEC);
        straight(11, -30);
        turn(90, -30, 30);
    }
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30, false);
    straight(6.5, 30);
    turn(90, -30, 30);
    linetrace_cm_pd_SP(17.5, 30, false);
    straight(10, 40);