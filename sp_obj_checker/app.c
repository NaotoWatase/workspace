//surprise
void sp_know(int num);
//surprise
void sp_check(int num, way_t sensor);
//surprise
void sp_checker(int num);

object_t check_result;

int sp_obj_count = 0;

void sp_obj_check(int num, way_t sensor){
    obj_measure(num, sensor);
    sp_obj_know(num, sensor);
}

void sp_obj_know(int num, way_t sensor){
    if (sensor == RIGHT){
        switch (obj){
            case 4:
            case 5:
            case 6:
                location[num] = /*object*/;
                check_result = /*object*/;
                break;
            default:
                location[num] = /*object*/;
                check_result = /*object*/;
                break;
        }
    }
    //基本こっちなはず
    if (sensor == LEFT){
        switch (obj){
            case 4:
            case 5:
            case 6:
                location[num] = /*object*/;
                check_result = /*object*/;
                break;
            default:
                location[num] = /*object*/;
                check_result = /*object*/;
                break;
        }
    }   
}

void sp_obj_checker(int num){
    if (location[num] == /*object*/ || /*location[num] == object*/){
        if (sp_obj_count == 1){
            switch (num){
                case 3:
                    straight(5.6, -50, false, false);
                    turn(90, 80, -80);
                    straight(2, 50, false, false);
                    sp_obj_check(num, LEFT);
                    steering_time(300, 15, 0);
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
                    break;
                case 9:
                case 10:
                    sp_obj_check(num, RIGHT);
                    /*program*/
                    break;
                default:
                    turn(180, -80, 80);
                    straight(22, -80, false, false);
                    sp_obj_check(num, LEFT);
                    straight(22, 80, false, false);
                    turn(180, 80, -80);
                    break;
            }
            sp_obj_count = 2;
        }
        else {
            if (check_result == /*object*/) location[num] = /*object*/;
            else location[num] = /*object*/;
        }
    }
}
