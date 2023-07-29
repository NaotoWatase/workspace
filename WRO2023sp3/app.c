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
//!!!!!!!!!!!!!!!!!smallshipは、turn2個でずらす動きをする時に壁にぶつかるので、smallshipのいちをずらす必要がある!!!!!!!!!!!!!!!!!!!!


//具体的な流れ

//startからwhite取りいくまで
start_nkc();

arm_reset_A();
linetrace_cm_pd_SP(38, 30);
linetrace_color_pd_SP(BOTH, COLOR_BLACK, 20);
straight(6.5, 30);
turn(90, -30, 30);
arm_set_A(80, false);
linetrace_cm_pd_SP(19, 30, false);
linetrace_cm_pd_SP(16.5, 60, true);
tslp_tsk(100*MSEC);
arm_set_A(45, true);
tslp_tsk(100*MSEC);
arm_take_obj();
//armでオブジェクトとる
straight(35.5, -60);
turn(90, -30, 30);

//4patternの前に戻ってくる
arm_reset_A();
linetrace_cm_pd_SP(10, 20);
linetrace_color_pd_SP(BOTH, COLOR_BLACK, 25);
linetrace_cm_pd_SP(6.5, 30);
turn(90, 30, -30);

objprepare_nkc();


//objprepare_nkcの中身
    linetrace_cm_pd_SP(4, 20, false);
    linetrace_cm_pd_SP(15, 30, false);
    linetrace_cm_pd_SP(8.5, 10, true);
    tslp_tsk(200*MSEC);
    turn(180, 65, 0);
    tslp_tsk(100*MSEC);

    straight(1.5, 30);
    obj_check(4, LEFT);
    straight_on(-20);
    straight_off(7.2, false);
    obj_check(3, LEFT);
    /*
    while (true){
        if(ev3_color_sensor_get_reflect(EV3_PORT_3) > 60 && ev3_color_sensor_get_reflect(EV3_PORT_2) > 60) break;
    }
    while (true){
        if(ev3_color_sensor_get_reflect(EV3_PORT_3) < 10 && ev3_color_sensor_get_reflect(EV3_PORT_2) < 10) break;
    }
    */
    while (true){
        if(ev3_color_sensor_get_color(EV3_PORT_3) == COLOR_WHITE && ev3_color_sensor_get_color(EV3_PORT_2) == COLOR_WHITE) break;
    }
    while (true){
        if(ev3_color_sensor_get_color(EV3_PORT_3) ==COLOR_BLACK && ev3_color_sensor_get_color(EV3_PORT_2) == COLOR_BLACK) break;
    }
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    obj_check(2, LEFT);
    straight_off(0.7, true);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    tslp_tsk(100*MSEC);
    location[5] = 10 - location[4] - location[3] - location[2]; //SPRZ
    location_f[3] = location[5];


