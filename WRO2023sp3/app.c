//big_shipのarm下ろす方法。
    if (arm_place[0] =! target_1){
        arm_mode_change(RIGHTDOWNRIGHT);
        arm_mode_change(LEFTDOWN);
        turn(80, 0, 30);
        turn(80, 30, 0);
        straight(12, -30);
        arm_mode_change(RIGHTDOWNRIGHT);
        turn(80, 30, 0);
        turn(80, 0, 30);
    }
    else {
        arm_mode_change(LEFTDOWNLEFT);
        arm_mode_change(RIGHTDOWN);
        turn(80, 30, 0);
        turn(80, 0, 30);
        straight(12, -30);
        arm_mode_change(LEFTDOWNLEFT);
        turn(80, 0, 30);
        turn(80, 30, 0);
    }
    if(arm_place[2] == surprise_object){
        turn(80, 0, 30);
        turn(80, 30, 0);
        straight(24, -30);
        arm_mode_change(RIGHTDOWN);
        arm_mode_change(LEFTDOWN);
        turn(80, 30, 0);
        turn(80, 0, 30);
    }
    else {
        turn(90, -30, 30);
        straight(10, 30);
        turn(90, 30, -30);
        straight(12, -30);
        arm_mode_change(LEFTDOWN);
        arm_mode_change(RIGHTDOWN);
        straight(12, 30);
        turn(90, 30, -30);
        straight(10, 30);
        turn(90, -30, 30);
    }
//small_shipも同じように作ってね！！


