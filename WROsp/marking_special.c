<<<<<<< HEAD
//手前にずらすプログラム

void marking_nkc(){
    map_decide();
    //marking
    steering_time(800, 10, 12);
    steering_time(500, 12, 30);
    straight(38, -80, false, false);
    steering_time(800, -20, 0);
    tslp_tsk(100*MSEC);
    straight(3, 40, false, false);
    turn(180, 80, 0);
    tslp_tsk(100*MSEC);
    straight(20, -50, false, false);
    steering_time(1000, -20, 0);
    tslp_tsk(300*MSEC);
    straight(48, 80, false, false);
    turn(90, 80, -80);
    straight(6, -40, false, false);
    marking_overall(140, 30);
    marking_overall(160, 10);
    straight(6, 40, false, false);
    turn(90, 50, -50);
    straight(30, -80, false, false);
    turn(270, 50, -50);
    straight(6, -50, false, false);
    marking_overall(0, -50);
    straight(6, 50, false, false);
    turn(90, 80, -80);
    straight(70, 80, false, false);
    steering_time(1000, 10, 30);
    turn(180, 0, -80);
    straight(30, 80, false, false);

    steering_time(800, 10, 3);
    steering_time(500, 12, 3);
    steering_time(300, 10, -3);
    tslp_tsk(100*MSEC);
    straight(2, -25, false, false);
    turn(180, -80, 0);
    tslp_tsk(100*MSEC);
    straight(7, 50, false, false);
    steering_time(1000, 15, 0);
    tslp_tsk(300*MSEC);
    straight(22, -80, false, false);
    //get the first block
    marking_overall(140, 30);
    marking_overall(160, 10);
    tslp_tsk(100*MSEC);
    straight(4, 50, false, false);
    tslp_tsk(300*MSEC);
    //往路
    if (map[BROWN] == 1) marking_long();
    if (map[RED] == 1) marking_short();
    if(map[YELLOW] == 1 || map[WHITE] == 1){
        straight(14.5, -80, true, false);
        tslp_tsk(200*MSEC);
        if (map[WHITE] == 1) marking_long();
        if (map[YELLOW] == 1) marking_short();
        straight(14.5, -80, true, false);
    }
    else{
        if(map[BLUE] == 1){
            straight(14, -80, true, false);
            tslp_tsk(100*MSEC);
            turn(36, -30, 0);
            tslp_tsk(100*MSEC);
            marking_overall(180, 10);
            marking_overall(220, 54);
            ev3_motor_rotate(EV3_PORT_D, 120, -50, true);
            tslp_tsk(100*MSEC);
            turn(36, 30, 0);
            tslp_tsk(100*MSEC);
            straight(15, -60, false, false);
        }
        else straight(29, -80, true, false);
    }
    marking_overall(165, 10);
    //復路
    straight(2, 50, false, false);//場所をそろえる
    if (map[GREEN] == 2) marking_short();
    if (map[BLUE] == 2) marking_long();
    if(map[YELLOW] == 2 || map[WHITE] == 2){
        straight(14.5, 80, false, false);
        tslp_tsk(200*MSEC);
        if (map[YELLOW] == 2) marking_short();
        if (map[WHITE] == 2) marking_long();
        straight(14.5, 80, false, false);
    }
    else{
        straight(29, 80, false, false);
        if(map[RED] == 2) marking_short();
    }

    straight(35, -80, false, false);
    marking_overall(140, 30);
    marking_overall(160, 10);
    straight(35, 80, false, false);
    straight(5, -50, false, false);
    if (map[BROWN] == 3) marking_long();
    if (map[RED] == 3) marking_short();
    straight(14.5, -80, false, false);
    if (map[WHITE] == 3) marking_long();
    if (map[YELLOW] == 3) marking_short();
    straight(14.5, -80, false, false);
    if (map[GREEN] == 3) marking_short();
    if (map[BLUE] == 3) marking_long();
    straight(30, 80, false, false);
}


//３つめはパーソンでないなにかにしないといけない
void map_decide(){
    if (location[8] == PERSON || location[9] == PERSON) {
        map[BROWN] = 1;
        marking_count = 1;
    }
    if (location[10] == PERSON || location[11] == PERSON) {
        if(marking_count == 1) map[RED] = 2;
        else {
            map[RED] = 1;
            marking_count = 1;
        }   
    }
    if (location[5] == PERSON || location[6] == PERSON) {
        if(marking_count == 1) map[WHITE] = 2;
        else {
            map[WHITE] = 1;
            marking_count = 1;
        }   
    }  
    if (location[4] == PERSON || location[7] == PERSON) {
        if(marking_count == 1) map[YELLOW] = 2;
        else {
            map[YELLOW] = 1;
            marking_count = 1;
        }   
    } 
    if (location[0] == PERSON || location[1] == PERSON) {
        if(marking_count == 1) map[BLUE] = 2;
        else {
            map[BLUE] = 1;
            marking_count = 1;
        }   
    }  
    if (location[2] == PERSON || location[3] == PERSON) {
        if(marking_count == 1) map[GREEN] = 2;
        else {
            map[GREEN] = 1;
            marking_count = 1;
        }   
    }
    if (location[8] == /*object*/ || location[9] == /*object*/) map[BROWN] = 3;
    if (location[10] == /*object*/ || location[11] == /*object*/) map[RED] = 3;
    if (location[5] == /*object*/ || location[6] == /*object*/) map[WHITE] = 3;
    if (location[4] == /*object*/ || location[7] == /*object*/) map[YELLOW] = 3;
    if (location[0] == /*object*/ || location[1] == /*object*/) map[BLUE] = 3;
    if (location[2] == /*object*/ || location[3] == /*object*/) map[GREEN] = 3;
=======
//手前にずらすプログラム

void marking_nkc(){
    map_decide();
    //marking
    steering_time(800, 10, 12);
    steering_time(500, 12, 30);
    straight(38, -80, false, false);
    steering_time(800, -20, 0);
    tslp_tsk(100*MSEC);
    straight(3, 40, false, false);
    turn(180, 80, 0);
    tslp_tsk(100*MSEC);
    straight(20, -50, false, false);
    steering_time(1000, -20, 0);
    tslp_tsk(300*MSEC);
    straight(48, 80, false, false);
    turn(90, 80, -80);
    straight(6, -40, false, false);
    marking_overall(140, 30);
    marking_overall(160, 10);
    straight(6, 40, false, false);
    turn(90, 50, -50);
    straight(30, -80, false, false);
    turn(270, 50, -50);
    straight(6, -50, false, false);
    marking_overall(0, -50);
    straight(6, 50, false, false);
    turn(90, 80, -80);
    straight(70, 80, false, false);
    steering_time(1000, 10, 30);
    turn(180, 0, -80);
    straight(30, 80, false, false);

    steering_time(800, 10, 3);
    steering_time(500, 12, 3);
    steering_time(300, 10, -3);
    tslp_tsk(100*MSEC);
    straight(2, -25, false, false);
    turn(180, -80, 0);
    tslp_tsk(100*MSEC);
    straight(7, 50, false, false);
    steering_time(1000, 15, 0);
    tslp_tsk(300*MSEC);
    straight(22, -80, false, false);
    //get the first block
    marking_overall(140, 30);
    marking_overall(160, 10);
    tslp_tsk(100*MSEC);
    straight(4, 50, false, false);
    tslp_tsk(300*MSEC);
    //往路
    if (map[BROWN] == 1) marking_long();
    if (map[RED] == 1) marking_short();
    if(map[YELLOW] == 1 || map[WHITE] == 1){
        straight(14.5, -80, true, false);
        tslp_tsk(200*MSEC);
        if (map[WHITE] == 1) marking_long();
        if (map[YELLOW] == 1) marking_short();
        straight(14.5, -80, true, false);
    }
    else{
        if(map[BLUE] == 1){
            straight(14, -80, true, false);
            tslp_tsk(100*MSEC);
            turn(36, -30, 0);
            tslp_tsk(100*MSEC);
            marking_overall(180, 10);
            marking_overall(220, 54);
            ev3_motor_rotate(EV3_PORT_D, 120, -50, true);
            tslp_tsk(100*MSEC);
            turn(36, 30, 0);
            tslp_tsk(100*MSEC);
            straight(15, -60, false, false);
        }
        else straight(29, -80, true, false);
    }
    marking_overall(165, 10);
    //復路
    straight(2, 50, false, false);//場所をそろえる
    if (map[GREEN] == 2) marking_short();
    if (map[BLUE] == 2) marking_long();
    if(map[YELLOW] == 2 || map[WHITE] == 2){
        straight(14.5, 80, false, false);
        tslp_tsk(200*MSEC);
        if (map[YELLOW] == 2) marking_short();
        if (map[WHITE] == 2) marking_long();
        straight(14.5, 80, false, false);
    }
    else{
        straight(29, 80, false, false);
        if(map[RED] == 2) marking_short();
    }

    straight(35, -80, false, false);
    marking_overall(140, 30);
    marking_overall(160, 10);
    straight(35, 80, false, false);
    straight(5, -50, false, false);
    if (map[BROWN] == 3) marking_long();
    if (map[RED] == 3) marking_short();
    straight(14.5, -80, false, false);
    if (map[WHITE] == 3) marking_long();
    if (map[YELLOW] == 3) marking_short();
    straight(14.5, -80, false, false);
    if (map[GREEN] == 3) marking_short();
    if (map[BLUE] == 3) marking_long();
    straight(30, 80, false, false);
}


//３つめはパーソンでないなにかにしないといけない
void map_decide(){
    if (location[8] == PERSON || location[9] == PERSON) {
        map[BROWN] = 1;
        marking_count = 1;
    }
    if (location[10] == PERSON || location[11] == PERSON) {
        if(marking_count == 1) map[RED] = 2;
        else {
            map[RED] = 1;
            marking_count = 1;
        }   
    }
    if (location[5] == PERSON || location[6] == PERSON) {
        if(marking_count == 1) map[WHITE] = 2;
        else {
            map[WHITE] = 1;
            marking_count = 1;
        }   
    }  
    if (location[4] == PERSON || location[7] == PERSON) {
        if(marking_count == 1) map[YELLOW] = 2;
        else {
            map[YELLOW] = 1;
            marking_count = 1;
        }   
    } 
    if (location[0] == PERSON || location[1] == PERSON) {
        if(marking_count == 1) map[BLUE] = 2;
        else {
            map[BLUE] = 1;
            marking_count = 1;
        }   
    }  
    if (location[2] == PERSON || location[3] == PERSON) {
        if(marking_count == 1) map[GREEN] = 2;
        else {
            map[GREEN] = 1;
            marking_count = 1;
        }   
    }
    if (location[8] == /*object*/ || location[9] == /*object*/) map[BROWN] = 3;
    if (location[10] == /*object*/ || location[11] == /*object*/) map[RED] = 3;
    if (location[5] == /*object*/ || location[6] == /*object*/) map[WHITE] = 3;
    if (location[4] == /*object*/ || location[7] == /*object*/) map[YELLOW] = 3;
    if (location[0] == /*object*/ || location[1] == /*object*/) map[BLUE] = 3;
    if (location[2] == /*object*/ || location[3] == /*object*/) map[GREEN] = 3;
>>>>>>> ea9d688f7dc8b77813353c1b5f66e29226af3d6b
}