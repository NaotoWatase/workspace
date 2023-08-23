スタートの処理。最初に25cmに入る必要があるため

    now_arm_angle = 0;
    ev3_motor_set_power(EV3_PORT_D, 20);
    tslp_tsk(800*MSEC);
    //ev3_motor_stop(EV3_PORT_D, true);
    ev3_motor_reset_counts(EV3_PORT_D);



arm_take_shipの動き

    arm_mode_change_D_SPRZ(UP);
    tslp_tsk(500*MSEC);
    straight(15, 30);
    straight(1.5, -30);
    arm_mode_change_D_SPRZ(DOWN);
    turn(90, -30, 30);
    straight(10,30);
    arm_mode_change_D_SPRZ(UP);
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