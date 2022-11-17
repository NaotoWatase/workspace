//surprise
void sp_know(int num);
//surprise
void sp_obj_check(int num);
//surprise
void sp_obj_checker(int num);

object_t chemical_result;

//straight の arm_up() を arm_normal() にすべし

//それぞれのエリアの分岐をchemical = 1から2に変えるべし

void chemical_taker(int n, way_t sensor){
    timing_chemical = 0;
    if(location[n] == CHEMICAL && chemical == 1){
        timing_chemical = 1;
        chemical = 2;
    }
    if(location[n] == CHEMICAL && chemical == 0){
        chemical = 1;
    }
}

void sp_obj_check(int num){
    obj_measure(num, LEFT);
    sp_obj_know(num);
}

void sp_obj_know(int num){
    switch (obj){
        case 4:
        case 5:
        case 6:
            location[num] = /*object*/;
            chemical_result = /*object*/;
            break;
        default:
            location[num] = /*object*/;
            chemical_result = /*object*/;
            break;
    }
}

void sp_obj_checker(int num){
    if (location[num] == CHEMICAL || /*location[num] == object*/){
        if (chemical == 1){
            switch (num){
                case 1:
                case 7:
                    tslp_tsk(300*MSEC);
                    turn(180, -80, 80);
                    tslp_tsk(300*MSEC);
                    straight(22, -80, false, false);
                    tslp_tsk(300*MSEC);
                    sp_obj_check(num, LEFT);
                    straight(3.3, 50, false, false);
                    arm_left_up();
                    tslp_tsk(500*MSEC);
                    chemical_type = LEFT;
                    straight(18.7, 80, false, false);
                    tslp_tsk(300*MSEC);
                    turn(180, 80, -80);
                    tslp_tsk(300*MSEC);
                    break;
                case 3:
                    straight(3, -50, false, false);
                    turn(90, 80, -80);
                    straight(4, 50, false, false);
                    sp_obj_check(num, LEFT);
                    steering_time(300, 15, 0);
                    arm_left_up();
                    chemical_type = LEFT;
                    tslp_tsk(500*MSEC);
                    straight(8, -50, false, false);
                    turn(90, -80, 80);
                    steering_time(500, 15, 0);
                    break;
                case 11:
                    straight(3, -50, false, false);
                    turn(90, 80, -80);
                    straight(4, 50, false, false);
                    sp_obj_check(num, LEFT);
                    steering_time(300, 15, 0);
                    straight(8, -50, false, false);
                    turn(90, -80, 80);
                    steering_time(500, 15, 0);
                    arm_right_up();
                    chemical_type = RIGHT;
                    tslp_tsk(500*MSEC);
                    break;
                default:
                    tslp_tsk(300*MSEC);
                    straight(22, 80, false, false);
                    tslp_tsk(300*MSEC);
                    turn(180, -80, 80);
                    tslp_tsk(300*MSEC);
                    sp_obj_check(num, LEFT);
                    straight(3.3, 50, false, false);
                    arm_left_up();
                    tslp_tsk(500*MSEC);
                    chemical_type = LEFT;
                    straight(18.7, 80, false, false);
                    tslp_tsk(300*MSEC);
                    turn(180, 80, -80);
                    tslp_tsk(200*MSEC);
                    break;
            }
        }
        else {
            //2個目はこっちで処理
            //基本はtiming_chemical を使って straight でとるつもり 
            switch (num){
                case 8:
                case 9:
                    if (chemical_type == LEFT){
                        turn(180, 80, -80);
                        straight(16, -80, false, false);
                        arm_normal();
                        tslp_tsk(500*MSEC);
                        straight(16, 80, false, false);
                        turn(180, -80, 80);
                    }
                    break;
                case 11:
                    if (chemical_type == RIGHT){
                        straight(6, -80, false, false);
                        ev3_motor_rotate(EV3_PORT_A, 160, -10, false);
                        tslp_tsk(1600*MSEC);
                        ev3_motor_rotate(EV3_PORT_A, 80, -8, false);
                        tslp_tsk(200*MSEC);
                        straight(6, 80, false, false);
                        steering_time(300, 30, 0);
                        arm_sptype = LEFT_UP;
                        arm_right_up();
                        tslp_tsk(700*MSEC);
                        straight(8, -80, false, false);
                        turn(180, -80, 80);
                        steering_time(500, -20, 0);
                        arm_normal();
                        tslp_tsk(700*MSEC);
                        straight(8, 80, false, false);
                        turn(180, 80, -80);
                        steering_time(800, 20, 0);
                    }
                    break:
                default:
                    if (chemical_type == RIGHT){
                        turn(180, -80, 80);
                        straight(16, -80, false, false);
                        arm_normal();
                        tslp_tsk(500*MSEC);
                        straight(16, 80, false, false);
                        turn(180, 80, -80);
                    }
                    break;
            }
            if (check_result == CHEMICAL) {
                location[num] = /*object*/;
                if (chemical_type == RIGHT) chemical_type = RIGHT;
                if (chemical_type == LEFT) chemical_type = LEFT;
            }
            else {
                location[num] = CHEMICAL;
                if (chemical_type == RIGHT) chemical_type = LEFT;
                if (chemical_type == LEFT) chemical_type = RIGHT;
            }
        }
    }
}