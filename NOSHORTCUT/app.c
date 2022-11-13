void red_nkc(){
    obj_check(10, RIGHT);
    chemical_taker(10, RIGHT);
    straight(27.7, 80, true, true);
    obj_check(11, RIGHT);
    steering_time(500, 30, -8);
    steering_time(200, 10, 3);
    tslp_tsk(100*MSEC);
    if (location[11] == CHEMICAL){
        if (arm_type == DOWN) ev3_motor_rotate(EV3_PORT_A, 176, 30, false);
        arm_type = UP;
        chemical = chemical + 1;
        chemical_type = RIGHT;
        chemical_taker(11, RIGHT);
    }
    tslp_tsk(600*MSEC);
    straight(8, -50, false, false);
    turn(180, -80, 80);
    straight(6, -80, false, false);
    steering_time(500, -30, 0);
    steering_time(200, -10, 0);
    tslp_tsk(200*MSEC);
    straight(16, 80, false, false);
        water(10);
        water(11);
    straight(33, 80, true, false);
    turn(90, 80, -80);
    tslp_tsk(400*MSEC);
    straight(7, -30, false, false);
    steering_time(1000, -20, 0);
    tslp_tsk(1000*MSEC);
    straight(22.7, 80, false, false);
}

void brown_nkc(){
        turn(180, -60, 0);
        tslp_tsk(200*MSEC);
        straight(29.4, -80, false, false);
        obj_check(8, RIGHT);
        tslp_tsk(200*MSEC);
        straight(9.4, -80, false, false);
        steering_time(600, -20, 0);
        straight(12, 80, false, false);
        turn(90, 80, -80);
        tslp_tsk(200*MSEC);
            straight(25, -50, false, false);
            steering_color(COLOR_RED, -25, 0);
            tslp_tsk(600*MSEC);
            straight(6.5, 50, false, false);
            obj_check(9, LEFT);
            chemical_taker(9, LEFT);
            straight(37, 80, false, false);
            chemical_taker(8, LEFT);
            straight(63, 80, false, false);

}