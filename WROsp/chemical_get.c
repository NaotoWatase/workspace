//surprise
void sp_know(int num);
//surprise
void sp_check(int num, way_t sensor);
//surprise
void sp_checker(int num);

object_t chemical_result;

//straight の arm_up() を arm_normal() にすべし

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

void sp_chemical_check(int num){
    obj_measure(num, LEFT);
    sp_chemical_know(num);
}

void sp_chemical_know(int num){
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

void sp_chemical_get(int num){
    if (location[num] == CHEMICAL || /*location[num] == object*/){
        if (chemical == 1){
            switch (num){
                case 3:
                    straight(5.6, -50, false, false);
                    turn(90, 80, -80);
                    straight(2, 50, false, false);
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
                    straight(5.6, -50, false, false);
                    turn(90, 80, -80);
                    straight(2, 50, false, false);
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
                    turn(180, -80, 80);
                    straight(22, -80, false, false);
                    sp_obj_check(num, LEFT);
                    straight(3.3, 50, false, false);
                    arm_left_up();
                    tslp_tsk(500*MSEC);
                    chemical_type = LEFT;
                    straight(22, 80, false, false);
                    turn(180, 80, -80);
                    break;
            }
        }
        else {
            //2個目はこっちで処理
            switch (num){
                case 8:
                case 9:
                    if (chemical_type == LEFT){
                        turn(180, 80, -80);
                        straight(14, -80, false, false);
                        arm_normal();
                        tslp_tsk(500*MSEC);
                        straight(14, 80, false, false);
                        turn(180, -80, 80);
                    }
                    break;
                default:
                    if (chemical_type == RIGHT){
                        turn(180, -80, 80);
                        straight(14, -80, false, false);
                
                        arm_normal();
                        tslp_tsk(500*MSEC);
                        straight(14, 80, false, false);
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