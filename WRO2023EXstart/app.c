船を押して、マーキングを読むversion
    straight(8, 30);
    turn(45, 0, 30);
    turn(45, 30, 0);
    linetrace_color_pd_SP(RIGHT, COLOR_BLACK, 24, false); //p=24でrightセンサーだけのライントレース
    straight(10.6, 30);
    straight_on(-20); 
    straight_off(1.1, false);
    obj_check(1, LEFT);
    straight_off(5, false);
    obj_check(0, LEFT);
    straight_off(2, true);

    turn(90, 80, -80);

いきなり船掴むversion EX2に続き有り
    straight_on(-30);
    tslp_tsk(600*MSEC);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);

    arm_take_obj();

    straight(32, 30);
    turn(90, -30, 30);

    straight_on(-30);
    tslp_tsk(800*MSEC);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);

    straight_on(30);
    while (true){
        if(ev3_color_sensor_get_color(EV3_PORT_2) == COLOR_WHITE && ev3_color_sensor_get_color(EV3_PORT_3) == COLOR_WHITE) break;
    }
    arm_set_A(-110, false);
    straight(31, 30);
    turn(90, -30, 30);

    
    straight(8, 30);
    straight_off(4, true);
    tslp_tsk(200*MSEC);
    ev3_motor_set_power(EV3_PORT_A, 60);