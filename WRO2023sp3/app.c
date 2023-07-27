//objectの指定
void check_pattern(){
    int line2_1_count = 1;
    int line2_2_count = 1;
    line2_1 = location_t[0];
    line2_2 = surprise_object;
    if (location_f[0] == line2_1 && line2_1_count == 1) {
        line2_1_count = line2_1_count - 1;
        location_aim[0] = 2;
    }
    else if (location_f[0] == line2_2 && line2_2_count == 1) {
        line2_2_count = line2_2_count - 1;
        location_aim[0] = 2;
    }
    if (location_f[1] == line2_1 && line2_1_count == 1) {
        line2_1_count = line2_1_count - 1;
        location_aim[1] = 2;
    }
    else if (location_f[1] == line2_2 && line2_2_count == 1) {
        line2_2_count = line2_2_count - 1;
        location_aim[1] = 2;
    }
    if (location_f[2] == line2_1 && line2_1_count == 1) {
        line2_1_count = line2_1_count - 1;
        location_aim[2] = 2;
    }
    else if (location_f[2] == line2_2 && line2_2_count == 1) {
        line2_2_count = line2_2_count - 1;
        location_aim[2] = 2;
    }
    if (location_f[3] == line2_1 && line2_1_count == 1) {
        line2_1_count = line2_1_count - 1;
        location_aim[3] = 2;
    }
    else if (location_f[3] == line2_2 && line2_2_count == 1) {
        line2_2_count = line2_2_count - 1;
        location_aim[3] = 2;
    }
    pattern = location_aim[0] * 1000 + location_aim[1] * 100 + location_aim[2] * 10 + location_aim[3];
}


//big_shipのarm下ろす方法。
    if (arm_place[0] == location_t[1]){
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


