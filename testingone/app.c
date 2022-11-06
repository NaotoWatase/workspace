
/**
 * This sample program shows a PID controller for line following.
 *
 * Robot construction: Educator Vehicle
 *
 * References:
 * http://robotsquare.com/wp-content/uploads/2013/10/45544_educator.pdf
 * http://thetechnicgear.com/2014/03/howto-create-line-following-robot-using-mindstorms/
 */

#include "ev3api.h"
#include "app.h"
#include "motor.h"
#include "stdlib.h"

#include <stdio.h>
#include"math.h"

/* /dev/tty.MindstormsEV3-SerialPor */

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

static FILE *bt = NULL;



void sensor_check(uint8_t num);
void obj_measure(int num, way_t sensor);
void obj_know(int num);
void stopping();
void chemical_taker(int n, way_t sensor);
void obj_check(int num, way_t sensor);
void water(int n);
//直進
void straight(float cm, float set_power_sign, bool_t savedata);
//ac,dcで全体を一とした時の加速、減速の割合を変更
void straight_custom(float cm, float ac, float dc, float set_power);
void steering_time(int time_stop_4d, int power, int steering);
void turn(float angle, float L_power, float R_power);
void walltrace_length(float cm, float power, float distance);
void linetrace_length(float length, int power);
void steering_color(colorid_t color_stop, int power, int steering);
void marking_overall(int degree, int power);
void marking_short();
void marking_long();
//mapを定義 marking_countは優先度が高いものが１、低いものは２
void map_decide();
void arm_up();
void arm_down();


void start_nkc();
void blue_nkc();
void green_nkc();
void yellow_nkc();
void red_nkc();
void brown_nkc();
void white_nkc();
void chemical_nkc();
void marking_nkc();
void goal_nkc();
/**
 * Define the connection ports of the sensors and motors.
 * By default, this application uses the following ports:
 * Touch sensor: Port 2
 * Color sensor: Port 3
 * Left motor:   Port B
 * Right motor:  Port C
 */

static const sensor_port_t  PortUltrasonic = EV3_PORT_4;
static const sensor_port_t  PortSensorColor2 = EV3_PORT_2;
static const sensor_port_t  PortSensorColor3 = EV3_PORT_3;
static const sensor_port_t  PortSensorColor1 = EV3_PORT_1;
static const motor_port_t   PortMotorLeft   = EV3_PORT_B;
static const motor_port_t   PortMotorRight  = EV3_PORT_C;
static const motor_port_t   PortMotorArm1   = EV3_PORT_A;
static const motor_port_t   PortMotorArm2  = EV3_PORT_D;


static void button_clicked_handler(intptr_t button) {
    switch(button) {
    case BACK_BUTTON:
#if !defined(BUILD_MODULE)
        syslog(LOG_NOTICE, "Back button clicked.");
#endif
        break;
    }
}
bool_t birth;


int power = 50;

int timing_chemical = 0;

int judgement_check = 0;
int chemical = 0;
int marking_count = 0;

int white = 0;
int how_many = 31;

map_t map [6] = {0,0,0,0,0,0};
int start;

int chemical_type = 0;

int water_count = 1;

int y = 0;

float left_data = 0;
float right_data = 0;



arm_t arm_type = DOWN;


uint8_t obj = 0;
uint8_t test = 0;
rgb_raw_t testrgb;


int location[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0};

/*マルチタスク lovation_task用*/
int savedate_color[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0};
int savedate_red[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0};
int savedate_green[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0};
int savedate_blue[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0};
int location_counts = 0;
int task_counts = 0;

int location_sensor[12] = {0,0,0,0,0,0,0,0,0,0,0,0};


rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)


float h = 0;
float s = 0;
float v = 0;
float min = 0;
float max = 0;
float red = 0;
float green = 0;
float blue = 0;
float judgement = 0;
float obj_distance = 0;

int check_type;



void start_nkc() {
    ev3_motor_reset_counts(EV3_PORT_D);
    if(start == 1){
        straight(92, 100, false);
        turn(180, 0, -20);
        straight(70, -100, false);
    }
    if(start == 2){
        straight(80, -100, false);
    }
    turn(90, 50, -50);
    steering_time(700, -30, 0);
    straight(11.8, 50, false);
    //waltrace_length(12, 30, 10);
    turn(90, 50, -50);
    steering_color(COLOR_WHITE, 30, 0);
    steering_color(COLOR_BLACK, 24, 0);
    linetrace_length(28.5, 6);
    straight(6.8, 20, true);
}

void blue_nkc() {
    obj_check(0, RIGHT);
    chemical_taker(0, RIGHT);
    straight(37, 80, true);
    obj_check(1, RIGHT);
    chemical_taker(1, RIGHT);
    water(0);
    water(1);
    straight(11, 80, true);
}

void green_nkc() {
    obj_check(2, RIGHT);
    chemical_taker(2, RIGHT);
    straight(37, 80, true);
    obj_check(3, RIGHT);

    steering_time(600, 15, 0);
    if (location[3] == CHEMICAL){
        straight(8, -50, false);
        turn(90, 50, -50);
        steering_time(800, 15, 0);
        chemical_taker(3, LEFT);
        straight(10, -50, false);
        turn(180, 50, -50);
        steering_time(1000, -15, 0);
    }
    else{
        tslp_tsk(100*MSEC);
        tslp_tsk(200*MSEC);
        turn(90, -25, 0);
        tslp_tsk(300*MSEC);
        turn(45, -50, 50);
        tslp_tsk(300*MSEC);
        steering_time(1000, -15, 0);
        tslp_tsk(200*MSEC);
    }

    straight(25, 80, false);
    water(2);
    water(3);
}

void yellow_nkc() {
    obj_check(4, RIGHT);
    chemical_taker(4, RIGHT);

    /* red */
    straight(37.5, 80, true);
    water(4);
}

void red_nkc(){
    obj_check(10, RIGHT);
    chemical_taker(10, RIGHT);
    straight(27, 80, true);
    obj_check(11, RIGHT);
    chemical_taker(11, RIGHT);
    steering_time(1100, 10, 0);
    tslp_tsk(600*MSEC);
    straight(5.6, -10, false);
    tslp_tsk(700*MSEC);
    turn(90, -50, 50);
    tslp_tsk(700*MSEC);
    water(10);
    water(11);

    /* brown */
    walltrace_length(20.7, 8, 9);
}

void brown_nkc(){
    obj_check(9, LEFT);
    chemical_taker(9, LEFT);
    straight(37, 80, false);
    obj_check(8, LEFT);
    chemical_taker(8, RIGHT);
    tslp_tsk(300*MSEC);
    straight(10, -50, false);
    tslp_tsk(200*MSEC);
    turn(90, -50, 50);
    tslp_tsk(300*MSEC);
    straight(3, -28, false);
    steering_time(1000, -30, 0);
    tslp_tsk(400*MSEC);

    /* white */
    straight(14, 20, false);
    tslp_tsk(400*MSEC);
    water(8);
    water(9);
    straight(36.5, 80, false);
    tslp_tsk(600*MSEC);
    turn(90, 30, -30);
    tslp_tsk(400*MSEC);
    straight(39, -80, false);
}

void white_nkc(){
    obj_check(7, RIGHT);
    chemical_taker(7, RIGHT);
    tslp_tsk(300*MSEC);
    straight(11, 80, true);
    obj_check(6, RIGHT);
    chemical_taker(6, RIGHT);
    tslp_tsk(300*MSEC);
    straight(37, 80, true);
    obj_check(5, RIGHT);
    chemical_taker(5, RIGHT);
    water(5);
    water(6);
}

void chemical_nkc(){
    tslp_tsk(300*MSEC);
    straight(63, 80, true);
    if(chemical_type == RIGHT){
        tslp_tsk(300*MSEC);
        turn(90, -50, 50);
        tslp_tsk(300*MSEC);
        straight(10, -50, false);
        arm_down();
        ev3_motor_set_power(EV3_PORT_A, -15);
        straight(50, -80, false);
        ev3_motor_stop(EV3_PORT_A, true);
        steering_time(700, -25, 0);
        turn(180, 0, 25);
    }
    else {
        tslp_tsk(300*MSEC);
        turn(90, 50, -50);
        tslp_tsk(300*MSEC);
        straight(10, -50, false);
        arm_down();
        ev3_motor_set_power(EV3_PORT_A, -15);
        straight(45, 80, false);
        ev3_motor_stop(EV3_PORT_A, true);
        steering_time(800, 25, 0);
        straight(15, -50, false);
        turn(180, 25, 0);
    }
    
    /* crossingB */
    tslp_tsk(200*MSEC);
    straight_custom(83, 1, 0, -100);
    straight(3, -50, false);
}

void marking_nkc(){
    map_decide();
    //marking
    steering_time(1000, -30, 5);
    tslp_tsk(100*MSEC);
    steering_time(520, 30, -15);
    turn(108, -50, 50);
    steering_time(1200, 15, 0);
    tslp_tsk(100*MSEC);
    straight(22, -60, false);
    //get the first block
    marking_overall(140, 30);
    marking_overall(150, 10);
    //往路
    if (map[BROWN] == 1) marking_long();
    if (map[RED] == 1) marking_short();
    if(map[YELLOW] == 1 || map[WHITE] == 1){
        straight(14.5, -50, true);
        if (map[WHITE] == 1) marking_long();
        if (map[YELLOW] == 1) marking_short();
        straight(14.5, -50, true);
    }
    else{
        if(map[BLUE] == 1){
            straight(14, -50, true);
            turn(30, -20, 0);
            tslp_tsk(100*MSEC);
            marking_overall(230, 80);
            tslp_tsk(100*MSEC);
            marking_overall(120, -40);
            turn(30, 20, 0);
            tslp_tsk(100*MSEC);
            straight(15, -50, false);
        }
        else straight(29, -80, true);
    }
    marking_overall(160, 10);
    //復路
    if (map[GREEN] == 2) marking_short();
    if (map[BLUE] == 2) marking_long();
    if(map[YELLOW] == 2 || map[WHITE] == 2){
        straight(14.5, 50, false);
        if (map[YELLOW] == 2) marking_short();
        if (map[WHITE] == 2) marking_long();
        straight(14.5, 50, false);
    }
    else{
        straight(29, 80, false);
        if(map[RED] == 2) marking_short();
    }
}

void goal_nkc(){
    marking_overall(180, 30);
    if(start == 1){
        straight(60, -80, false);
        turn(185, 0, -25);
        steering_time(1200, -30, -5);
    }
    if(start == 2){
        straight(13, 80, false);
        turn(185, 25, 0);
        steering_time(1200, -30, 5);
    }
}

void test_turn() { 
    turn(90, -20, 20);
    straight(15, 80, false);
    turn(90, -20, 20);
    straight(15, 80, false);
    turn(90, -20, 20);
    straight(15, 80, false);
    turn(90, -20, 20);
    straight(15, 80, false);
    turn(90, -20, 20);
    straight(15, 80, false);
    turn(90, -20, 20);
    straight(15, 80, false);
    turn(90, -20, 20);
    straight(15, 80, false);
    turn(90, -20, 20);
    straight(15, 80, false);
    turn(90, -20, 20);
    straight(15, 80, false);
    turn(90, -20, 20);
    straight(15, 80, false);
    turn(90, -20, 20);
    straight(15, 80, false);
    turn(90, -20, 20);
    straight(15, 80, false);
    turn(90, -20, 20);
    straight(15, 80, false);
    turn(90, -20, 20);
    straight(15, 80, false);
    turn(90, -20, 20);
    straight(15, 80, false);
    turn(90, -20, 20);

} 

void arm_up() {
    if (arm_type == DOWN) ev3_motor_rotate(EV3_PORT_A, 180, 30, false);
    arm_type = UP;

}

void arm_down() {
    if (arm_type == UP) ev3_motor_rotate(EV3_PORT_A, 180, -10, true);
    arm_type = DOWN;
}

void stopping(){
    //while(ev3_button_is_pressed(ENTER_BUTTON) == false) {}    
    //tslp_tsk(2000*MSEC);
}

void turn(float angle, float L_power, float R_power) {
    float L_sign = L_power / abs(L_power);
    float R_sign = R_power / abs(R_power);
    float changing_L = 0;
    float changing_R = 0;
    float changing_power = 0;
    float set_power;
    float left;
    float right;
    float average;
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    if (R_power == 0) {
        (void)ev3_motor_rotate(EV3_PORT_B, angle*TURN*ROBOT1CM, (int16_t)-L_power, true);
    } 
    if (L_power == 0) {
        (void)ev3_motor_rotate(EV3_PORT_C, angle*TURN*ROBOT1CM, (int16_t)R_power, true);
    }
    if (R_power != 0 && L_power != 0) {
        set_power = abs(L_power);
        while (true){
            left = ev3_motor_get_counts(EV3_PORT_B);
            right = ev3_motor_get_counts(EV3_PORT_C);
            left = abs(left);
            right = abs(right);
            average = (left + right) / 2.0; 
            if (average < angle*TURN*ROBOT1CM * 1 / 4) {
                changing_power = (set_power / (angle*TURN*ROBOT1CM * 1 / 4)) * average;
                if (changing_power < 3) changing_power = 3;
                changing_L = changing_power * L_sign;
                changing_R = changing_power * R_sign;
            }
            if (average >= angle*TURN*ROBOT1CM * 1 / 4) {
                changing_power = (-set_power / (angle*TURN*ROBOT1CM * 1 / 4)) * (average - (angle*TURN*ROBOT1CM * 1 / 4)) + set_power;
                if (changing_power < 8) changing_power = 8;
                changing_L = changing_power * L_sign;
                changing_R = changing_power * R_sign;
            }
            (void)ev3_motor_set_power(EV3_PORT_B, -changing_L);
            (void)ev3_motor_set_power(EV3_PORT_C, changing_R);
            if (angle * TURN * ROBOT1CM < average) break;
        }
        ev3_motor_stop(EV3_PORT_B, true);
        ev3_motor_stop(EV3_PORT_C, true);
    }
    tslp_tsk(100);
}

void straight(float cm, float set_power_sign, bool_t savedata) {
    int sign = set_power_sign / abs(set_power_sign);
    float set_power = abs(set_power_sign);
    float lb_power;
    float rc_power;
    float p_gein = -6;
    float d_gein = 0;
    if (set_power < 30) p_gein = -0.7;
    if (cm < 15) {
        p_gein = -0.7;
        set_power = 30;
    }
    if (set_power > 70) {
        set_power = 70;
    }
    if(sign < 0 ) {
        p_gein = -0.01;
    }
    float changing_power = 0;
    float left;
    float right;
    float average;
    float steer;
    float diff;
    float bibun;
    float power = 0;

    float minus = 0;

    float hensa = 0;
    float last_hensa = 0;


    if (savedata == false) {
        ev3_motor_reset_counts(EV3_PORT_B);
        ev3_motor_reset_counts(EV3_PORT_C);	
    }
    if (savedata == true) {
        if (left_data - right_data >= 0) minus = right_data - left_data;
        if (left_data - right_data < 0) minus = right_data - left_data;
        ev3_motor_reset_counts(EV3_PORT_B);
        ev3_motor_reset_counts(EV3_PORT_C);	
    }


    while (true){
        int count_chemical = 0;
        left = ev3_motor_get_counts(EV3_PORT_B); 
        right = ev3_motor_get_counts(EV3_PORT_C) + minus;
        left = abs(left);
        right = abs(right);
        diff = left - right;
        hensa = diff;
        bibun = (hensa - last_hensa) / 0.005;
        steer = diff * p_gein + bibun * d_gein;
        average = (left + right) / 2.0; 
        if (average < cm*ROBOT1CM * 1 / 4) {
            changing_power = (set_power / (cm*ROBOT1CM * 1 / 4)) * average;
            if (changing_power < 7) changing_power = 7;
            power = changing_power * sign;
        }
        if (average >= cm*ROBOT1CM * 1 / 4) {
            changing_power = (-set_power / (cm*ROBOT1CM * 3 / 4)) * (average - (cm*ROBOT1CM * 1 / 4)) + set_power;
            if (changing_power < 9) changing_power = 9;
            power = changing_power * sign;
        }
        if (average >= cm*ROBOT1CM * 2 / 4) {
            p_gein = -2;
        }
        if (average >= cm*ROBOT1CM * 3 / 4) {
            p_gein = -6;
        }
        if (marking_count >= 1) {
            p_gein = -6;
        }
        if(steer > 0) {
            lb_power = power;
            rc_power = power - (power * steer / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steer / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        (void)ev3_motor_set_power(EV3_PORT_B, lb_power);
        (void)ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (timing_chemical == 1 && (left / ROBOT1CM > 1.8) && count_chemical == 0) {
            count_chemical = 1;
            arm_up();
        }
        if (cm * ROBOT1CM < average) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    left_data = left;
    right_data = right;
    //tslp_tsk(5 * MSEC);
    last_hensa = hensa;
}

void last(float cm, float set_power, bool_t savedata) {
    if (cm < 10 && set_power > 50) { 
        set_power = 50;
    }
    if (set_power > 70) { 
        set_power = 70;
    }
    float lb_power;
    float rc_power;
    float power = 0;
    float left;
    float right;
    float difference;
    float gein = -0;
    float steer;
    float maxspeed_length = cm * 10.0 / 20.0;
    float decele_length = cm * 10.0 / 20.0;

    int count_chemical = 0;

    float minus = 0;

    

    float diff = 0.0006;    // 0.0008 
    if (set_power > 0) {
        diff = 0.0013;
        //gein = -11;
        //gein = -9;
        gein = -2;
    }
    if (set_power < 0) {
        diff = -0.0008;
        //gein = -4;

    }

    //ev3_gyro_sensor_reset(EV3_PORT_4);
    if (savedata == false) {
        ev3_motor_reset_counts(EV3_PORT_B);
        ev3_motor_reset_counts(EV3_PORT_C);	
    }
    if (savedata == true) {
        if (left_data - right_data >= 0) minus = right_data - left_data;
        if (left_data - right_data < 0) minus = right_data - left_data;
        ev3_motor_reset_counts(EV3_PORT_B);
        ev3_motor_reset_counts(EV3_PORT_C);	
    }

    
    power = diff;

    while(true){
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C) + minus;
        left = abs(left);
        right = abs(right);
        difference = left - right;
        steer = difference * gein;
        //power = (set_power / maxspeed_length) * (left / ROBOT1CM);
        //if (set_power < 0 && power >= -8) power = -8;
        //if (set_power > 0 && power <= 8) power = 8;
        power += diff; 
        if (set_power < 0 && power > -7) power = -7;
        if(steer > 0) {
            lb_power = power;
            rc_power = power - (power * steer / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steer / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        (void)ev3_motor_set_power(EV3_PORT_B, lb_power);
        (void)ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (power >= set_power && set_power > 0) break;
        if (power <= set_power && set_power < 0) break;
        if (left >= maxspeed_length * ROBOT1CM) break;
        if (timing_chemical == 1 && (left / ROBOT1CM > 1.4) && count_chemical == 0) {
            count_chemical = 1;
            arm_up();
        }
    }
    if (set_power < 0) {
        //gein = -0.5;
        gein = 0;
    }
    if (set_power > 0) {
        //gein = -1;
        gein = 0;
    }
    if (MODE == 0) {
        gein = 0;
    }
    while(true){
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C) + minus;
        left = abs(left);
        right = abs(right);
        difference = left - right;
        steer = difference * gein;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steer / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steer / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        (void)ev3_motor_set_power(EV3_PORT_B, lb_power);
        (void)ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (maxspeed_length * ROBOT1CM <= left) break;
        if (timing_chemical == 1 && (left / ROBOT1CM > 1.4) && count_chemical == 0) {
            count_chemical = 1;
            arm_up();
        }
    }
    if (set_power < 0) {
        //gein = -0.3;
        gein = 0;
    }
    if (set_power > 0) {
        diff = 0.0013;
        //gein = -3;
        gein = -2;
    }
    if (MODE == 0) {
        gein = -0.1;
    }
    set_power = power;
    while(true){
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C) + minus;
        left = abs(left);
        right = abs(right);
        difference = left - right;
        steer = difference * gein;
        power = (-set_power / decele_length) * ((left / ROBOT1CM ) - maxspeed_length) + set_power;
        if (set_power > 0 && power <= 8) {
            power = 8;
        }
        if (set_power < 0 && power >= -8) {
            power = -8;
        }
        //else power = 15;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steer / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steer / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        (void)ev3_motor_set_power(EV3_PORT_B, lb_power);
        (void)ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (cm * ROBOT1CM <= left) break;
        if (timing_chemical == 1 && (left / ROBOT1CM > 1.4) && count_chemical == 0) {
            count_chemical = 1;
            arm_up();
        }

    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    left_data = left;
    right_data = right;
}

void straight_custom(float cm, float ac, float dc, float set_power) {
    if (cm < 10 && set_power > 50) { 
        set_power = 50;
    }
    if (set_power > 70) { 
        set_power = 80;
    }
    float lb_power;
    float rc_power;
    float power = 0;
    float left;
    float right;
    float difference;
    float gein = -0.5;
    float steer;
    float maxspeed_length = cm * ac;
    float decele_length = cm * dc;
    

    float diff = 0.0006;    // 0.0008 
    if (set_power > 0) {
        diff = 0.0015;
        gein = -11;
    }
    if (set_power < 0) {
        diff = -0.0008;
        gein = -0.5;
    }
    //ev3_gyro_sensor_reset(EV3_PORT_4);
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);	
    power = diff;
    while(true){
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        difference = left - right;
        steer = difference * gein;
        power += diff; 
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steer / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steer / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        (void)ev3_motor_set_power(EV3_PORT_B, lb_power);
        (void)ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (power >= set_power && set_power > 0) break;
        if (power <= set_power && set_power < 0) break;
        if (left >= maxspeed_length * ROBOT1CM) break;
    }
    if (set_power < 0) {
        gein = -0.5;
    }
    while(true){
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        difference = left - right;
        steer = difference * gein;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steer / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steer / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        (void)ev3_motor_set_power(EV3_PORT_B, lb_power);
        (void)ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (maxspeed_length * ROBOT1CM <= left) break;
    }
    if (set_power < 0) {
        gein = -0.3;
    }
    if (set_power > 0) {
        diff = 0.0013;
        gein = -3;
    }
    set_power = power;
    while(true){
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        difference = left - right;
        steer = difference * gein;
        power = (-set_power / decele_length) * ((left / ROBOT1CM ) - maxspeed_length) + set_power;
        if (set_power > 0 && power <= 10) {
            power = 10;
        }
        if (set_power < 0 && power >= -10) {
            power = -10;
        }
        //else power = 15;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steer / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steer / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        (void)ev3_motor_set_power(EV3_PORT_B, lb_power);
        (void)ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (cm * ROBOT1CM <= left) break;

    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void water(int n) {
    if (location[n] == FIRE) {
        if(water_count == 1) {
            ev3_motor_rotate(EV3_PORT_D, 70 , 25, true);
            ev3_motor_rotate(EV3_PORT_D, 70 , -25, false);
        }
        if(water_count == 2) {
            ev3_motor_rotate(EV3_PORT_D, 155 , 25, true);
            ev3_motor_rotate(EV3_PORT_D, 155 , -25, false);
        }
        water_count = water_count + 1;
    }
}

void steering_time(int time_stop_4d, int power, int steering){
    if(steering > 0) {
    (void)ev3_motor_set_power(EV3_PORT_B, -power);
    (void)ev3_motor_set_power(EV3_PORT_C, power+(power*steering/50));
    }
    else {
    (void)ev3_motor_set_power(EV3_PORT_B, -(power-(power*steering/50)));
    (void)ev3_motor_set_power(EV3_PORT_C, power);
    }
    tslp_tsk(time_stop_4d * MSEC);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void steering_color(colorid_t color_stop, int power, int steering){
    colorid_t color;
    if(steering > 0) {
        (void)ev3_motor_set_power(EV3_PORT_B, -power);
        (void)ev3_motor_set_power(EV3_PORT_C, power+(power*steering/50));
    }
    else {
        (void)ev3_motor_set_power(EV3_PORT_B, -(power-(power*steering/50)));
        (void)ev3_motor_set_power(EV3_PORT_C, power);
    }
    while (color_stop != color) {
        color = ev3_color_sensor_get_color(EV3_PORT_1);
    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);    
}

void p_turn(int angle, int left_motor, int right_motor){
    angle = angle * 178 / 180;
    int turn_check = 0;
    int gyro = 0;
    int power;
    float accele_length = angle / 10 * 1;
    float maxspeed_length = angle / 10 * 2;
    ev3_gyro_sensor_reset(EV3_PORT_4);
    gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
    gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
    while (true) {
        gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        gyro = abs(gyro);
        power = (100 / (angle / 10 * 1)) * gyro;
        if(power < 20) power = 20;
        if(power > 80) power = 80;
        if (gyro > accele_length) break;

        if (left_motor == 0) {
            power = power * 1.3;
            (void)ev3_motor_set_power(EV3_PORT_C, -power * right_motor); 
        }
        else if (right_motor == 0) {
            power = power * 1.3;
            (void)ev3_motor_set_power(EV3_PORT_B, -power * left_motor);
        } 
        else {
            (void)ev3_motor_set_power(EV3_PORT_B, -power * left_motor);
            (void)ev3_motor_set_power(EV3_PORT_C, -power * right_motor); 
        }
    }
    while (true) {
        gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        gyro = abs(gyro);
        power = 80;
        if (gyro > maxspeed_length) break;

        if (left_motor == 0) {
            power = power * 1.3;
            (void)ev3_motor_set_power(EV3_PORT_C, -power * right_motor); 
        }
        else if (right_motor == 0) {
            power = power * 1.3;
            (void)ev3_motor_set_power(EV3_PORT_B, -power * left_motor);
        } 
        else {
            (void)ev3_motor_set_power(EV3_PORT_B, -power * left_motor);
            (void)ev3_motor_set_power(EV3_PORT_C, -power * right_motor); 
        }
    }
    while (true) {
        gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        gyro = abs(gyro);
        power = (angle - gyro) * 1;
        if (angle - gyro == 0) turn_check = turn_check + 1;

        if (left_motor == 0) {
            power = power * 1.3;
            if(power < 6 && power > 0) power = 6;
            if(power > -6 && power < 0) power = -6;
            (void)ev3_motor_set_power(EV3_PORT_C, -power * right_motor); 
        }
        else if (right_motor == 0) {
            power = power * 1.3;
            if(power < 6 && power > 0) power = 6;
            if(power > -6 && power < 0) power = -6;
            (void)ev3_motor_set_power(EV3_PORT_B, -power * left_motor);
        } 
        else {
            if(power < 6 && power > 0) power = 6;
            if(power > -6 && power < 0) power = -6;
            (void)ev3_motor_set_power(EV3_PORT_B, -power * left_motor);
            (void)ev3_motor_set_power(EV3_PORT_C, -power * right_motor); 
        }
        if (turn_check > 500) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    tslp_tsk(100*MSEC);

}
    
void tank_turn(float angle, int power_L, int power_R){
    if (power_R == 0) {
        (void)ev3_motor_set_power(EV3_PORT_B, -power_L);
        (void)ev3_motor_rotate(EV3_PORT_B, angle*TURN*ROBOT1CM, (int16_t)-power_L, true);
    } 
    if (power_L == 0) {
        (void)ev3_motor_set_power(EV3_PORT_B, power_R);
        (void)ev3_motor_rotate(EV3_PORT_C, angle*TURN*ROBOT1CM, (int16_t)power_R, true);
    }
    if (power_L != 0 && power_R != 0) {
        (void)ev3_motor_set_power(EV3_PORT_B, -power_L);
        (void)ev3_motor_set_power(EV3_PORT_B, power_R);
        (void)ev3_motor_rotate(EV3_PORT_B, angle*TURN*ROBOT1CM, (int16_t)-power_L, false);
        (void)ev3_motor_rotate(EV3_PORT_C, angle*TURN*ROBOT1CM, (int16_t)power_R, true);
        ev3_motor_stop(EV3_PORT_B, true);
        ev3_motor_stop(EV3_PORT_C, true);
    }
}

void linetrace_length(float length, int power){
    ev3_motor_reset_counts(EV3_PORT_C);
    ev3_motor_reset_counts(EV3_PORT_B);
    int kakudo_C; 
    int kakudo_B; 
    float average;
    float steer;
    float p = 0;
    float i = 0;
    float d = 0;
    float d2 = 0;
    int reflect;
    while (true) {
        kakudo_C = ev3_motor_get_counts(EV3_PORT_C);
        kakudo_B = ev3_motor_get_counts(EV3_PORT_B);
        kakudo_B = abs(kakudo_B);
        kakudo_C = abs(kakudo_C);
        average = (kakudo_B + kakudo_C) / 2;
        reflect = ev3_color_sensor_get_reflect(EV3_PORT_1);
        
        p = reflect - 25;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  p * -0.8 + i * 0 + d * 0; 
        if (length * ROBOT1CM < average) break;
        if(steer > 0) {
            (void)ev3_motor_set_power(EV3_PORT_B, -power);
            (void)ev3_motor_set_power(EV3_PORT_C, power+(power*steer/50));
        }
        else {
            (void)ev3_motor_set_power(EV3_PORT_B, -(power-(power*steer/50)));
            (void)ev3_motor_set_power(EV3_PORT_C, power);
        }

    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
}

void hsv(int num, way_t sensor) {
    /*ev3_speaker_play_tone(NOTE_A5, 100);
    tslp_tsk(300*MSEC);*/
    float h = 0;
    float s = 0;
    float v = 0;
    float min = 0;
    float max = 0;
    float red;
    float green;
    float blue;
    float judgement;
    while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_2, &test)) {
        ;
    }  
    while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_2, &obj)) {
        ;
    }  
    while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_2, &testrgb)) {
            ;
    }  
    while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_2, &rgb_val)) {
            ;
    }  
    
    red = rgb_val.r;
    green = rgb_val.g;
    blue = rgb_val.b;
    judgement = red + green + blue;

    if (red <= green && red <= blue) { 
        min = red;
    }
    if (green <= red && green <= blue) { 
        min = green;
    }
    if (blue <= red && blue <= green) { 
        min = blue;
    }
    
    if (red >= green && red >= blue) { 
        max = red;
        h = 60 * ((green - blue) / (max - min));
    }
    if (green >= red && green >= blue) { 
        max = green;
        h = 60 * ((blue - red) / (max - min)) + 120;
    }
    if (blue >= red && blue >= green) { 
        max = blue;
        h = 60 * ((red - green) / (max - min)) + 240;
    }

    s = (max - min) / max * 100;

    v = max / 255 * 100;

    location_sensor[num] = obj;

    fprintf(bt, "LOCATION = %d\r\nCOLOR = %d  RGB:%f,%f,%f = JUDGE:%f\r\nRESULT = %d\r\nH:%f\r\nS:%f\r\nV:%f\r\nmax:%f\r\nmin:%f\r\n-----------------\r\n", num, obj, red, green, blue, judgement, location[num], h, s, v, max, min);
    /*ev3_speaker_play_tone(NOTE_A5, 100);
    tslp_tsk(400*MSEC);*/
}

void walltrace_length(float cm, float power, float distance) {
    //power20を想定
    float get_distance;
    float lb_power;
    float rc_power;
    float steer;
    float average;
    float lb;
    float rc;
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    while (true) {
        lb = ev3_motor_get_counts(EV3_PORT_B);
        rc = ev3_motor_get_counts(EV3_PORT_C);
        lb = abs(lb);
        rc = abs(rc);
        average = (lb + rc) / 2;

        get_distance = ev3_ultrasonic_sensor_get_distance(EV3_PORT_4);
        steer = (distance - get_distance) * 6;
        if (steer > 0) {
            lb_power = power;
            rc_power = power + (power * steer / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power - (power * steer / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (average > cm * ROBOT1CM) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void wall_turn(int power_L, int power_R){
    //ある程度壁に近づけてから使うべし
    float now_distance;
    float old_distance;
    if (power_R == 0) {
        //(void)ev3_motor_rotate(EV3_PORT_B, angle*TURN*ROBOT1CM, (int16_t)-power_L, true);
    } 
    if (power_L == 0) {
    }
    if (power_L != 0 && power_R != 0) {
        now_distance = ev3_ultrasonic_sensor_get_distance(EV3_PORT_4);
        while (true){
            old_distance = now_distance;
            now_distance = ev3_ultrasonic_sensor_get_distance(EV3_PORT_4);
            (void)ev3_motor_set_power(EV3_PORT_B, -power_L);
            (void)ev3_motor_set_power(EV3_PORT_C, power_R);
            if (now_distance > old_distance)break;
        } 
        ev3_motor_stop(EV3_PORT_B, true);
        ev3_motor_stop(EV3_PORT_C, true);
        turn(35, -power_L, -power_R);
    }   
}

void chemical_taker(int n, way_t sensor){
    timing_chemical = 0;
    if(location[n] == CHEMICAL && chemical == 0){
        timing_chemical = 1;
        chemical = chemical + 1;
       if(sensor == RIGHT){
           chemical_type = RIGHT;
        }
        else{
            chemical_type = LEFT;
        }
    }
}

void chemical_took(int n, way_t sensor){
    if(location[n] == CHEMICAL){
       if(sensor == RIGHT){
           ev3_motor_rotate(EV3_PORT_A, 30, -15, false);
           chemical_type = RIGHT;
        }
        else{
            ev3_motor_rotate(EV3_PORT_A, 30, 15, false);
            chemical_type = LEFT;
        }
    }
}

void obj_check(int num, way_t sensor){
    obj_measure(num, sensor);
    obj_know(num);
    //tslp_tsk(300*MSEC);
}

void obj_measure(int num, way_t sensor) {
    check_type = 0;
    if(sensor == LEFT){
        while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_2, &test)) {
            ;
        }  
        while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_2, &obj)) {
            ;
        }  
        while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_2, &testrgb)) {
            ;
        }  
        while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_2, &rgb_val)) {
            ;
        }  
        if(start == 1){
            obj_distance = ev3_ultrasonic_sensor_get_distance(EV3_PORT_4);
            tslp_tsk(100);
            check_type = 1;
        }
    } 
    else{ 
        while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_3, &test)) {
            ;
        }  
        while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_3, &obj)) {
            ;
        }  
        while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_3, &testrgb)) {
            ;
        }
        while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_3, &rgb_val)) {
            ;
        }
        if(start == 2){
            obj_distance = ev3_ultrasonic_sensor_get_distance(EV3_PORT_4);
            tslp_tsk(100);
            check_type = 1;
        }  
    }
    
    red = rgb_val.r;
    green = rgb_val.g;
    blue = rgb_val.b;
    judgement = red + green + blue;

    if (red <= green && red <= blue) { 
        min = red;
    }
    if (green <= red && green <= blue) { 
        min = green;
    }
    if (blue <= red && blue <= green) { 
        min = blue;
    }
    
    if (red >= green && red >= blue) { 
        max = red;
        h = 60 * ((green - blue) / (max - min));
    }
    if (green >= red && green >= blue) { 
        max = green;
        h = 60 * ((blue - red) / (max - min)) + 120;
    }
    if (blue >= red && blue >= green) { 
        max = blue;
        h = 60 * ((red - green) / (max - min)) + 240;
    }

    s = (max - min) / max * 100;

    v = max / 255 * 100;

    
}

void obj_know(int num){
    if (check_type == 0){
        switch (obj){
            case 2:
            case 3:
            case 4:
            case 11:
            case 13:
            case 16:
                location[num] = PERSON;
                break;
            case 1:
            case 12:
            case 14:
            case 17:
                if (judgement > 100 || blue > 40 || green > 40 || red > 40 || (red - green - blue > 12) || (blue - green - red > 15)) {
                    location[num] = PERSON;
                    if ((red - green - blue > 20) || (h > 10 && h < 50)) location[num] = FIRE;    
                    if (s > 30) location[num] = PERSON;
                } 
                else {
                    judgement_check = judgement;
                    location[num] = NOTHING;
                }
                break;
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
            case 10:
            case 15:
                location[num] = FIRE;
                break;
            case 0:
                if (red > 15 && red > blue && red > green) location[num] = FIRE;
                else location[num] = CHEMICAL;
                break; 
            default:
                location[num] = NOTHING;
                break;
        }
    }
    if (check_type == 1){
        if (obj_distance < 7) {
            switch (obj){
                case 1:
                case 2:
                case 3:
                    location[num] = PERSON;
                    sensor_check(obj);
                    break;
                case 4:
                case 13:
                    location[num] = PERSON;
                    sensor_check(obj);
                    break;
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                case 10:
                case 15:
                    location[num] = FIRE;
                    sensor_check(obj);
                    break;
                case 0:
                case 11:
                case 12:
                case 14:
                case 16:
                case 17:
                    location[num] = CHEMICAL;
                    if (judgement > 80 || blue > 40 || green > 40 || red > 40 || (h < 50 && h > 11)) {
                        location[num] = PERSON;
                        if (red - green - blue > 20 || (h > 10 && h < 50)) location[num] = FIRE;    
                    }    
                    sensor_check(obj);
                    break; 
                default:
                    break;
            } 
        }
        else {
            location[num] = NOTHING;
        }
    }  
    fprintf(bt, "LOCATION = %d\r\nCOLOR = %d\r\nRGB:%f,%f,%f = JUDGE:%f\r\nHSV:%f,%f,%f = MAX:%f MIN:%f\r\nDISTANCE:%f\r\nRESULT = %d\r\n-----------------\r\n", num, obj, red, green, blue, judgement, h, s, v, max, min, obj_distance, location[num]);
}

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
}

void marking_overall(int degree, int power){    
    //　＋以外の角度は存在しない!
    int num;
        while(true){
            num = ev3_motor_get_counts(EV3_PORT_D);
            ev3_motor_set_power(EV3_PORT_D, power);
            if (power > 0 && num >= degree)break;
            if (power < 0 && num <= degree)break;
        }
        ev3_motor_stop(EV3_PORT_D, true);
}

void marking_long(){
    marking_overall(150, 10);
    marking_overall(220, 40);
    tslp_tsk(100*MSEC);
    marking_overall(100, -40);
}

void marking_short(){
    marking_overall(150, 10);
    marking_overall(222, 20);
    tslp_tsk(100*MSEC);
    marking_overall(100, -40);
}

void sensor_check(uint8_t num) {
    /*int ct = 0;
    while (ct < num) {
        ev3_speaker_play_tone(NOTE_C5, 100);
        tslp_tsk(500*1000);
        ct = ct + 1;
    }*/
    //play_wave(num);
}

void check_task(intptr_t unused){
    fprintf(bt, "\r\ny:%d", y);
}

void location_r_task(intptr_t unused){
    stp_cyc(LOCATION_R_CYC);
    while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_3, &test)) {
        ;
    }  
    while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_3, &obj)) {
        ;
    }  
    while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_3, &testrgb)) {
            ;
    }  
    while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_3, &rgb_val)) {
            ;
    }  
    savedate_color[location_counts] = obj;
    savedate_red[location_counts] = rgb_val.r;
    savedate_green[location_counts] = rgb_val.g;
    savedate_blue[location_counts] = rgb_val.b;
    location_counts = location_counts + 1;
    task_counts = task_counts + 1;
}

void location_l_task(intptr_t unused){
    stp_cyc(LOCATION_L_CYC);
    while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_2, &test)) {
        ;
    }  
    while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_2, &obj)) {
        ;
    }  
    while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_2, &testrgb)) {
            ;
    }  
    while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_2, &rgb_val)) {
            ;
    }  
    savedate_color[location_counts] = obj;
    savedate_red[location_counts] = rgb_val.r;
    savedate_green[location_counts] = rgb_val.g;
    savedate_blue[location_counts] = rgb_val.b;
    location_counts = location_counts + 1;
    task_counts = task_counts + 1;
}

void main_task(intptr_t unused){

    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);



    
    /* Register button handlers */
    ev3_button_set_on_clicked(BACK_BUTTON, &button_clicked_handler, BACK_BUTTON);

    /* Configure motors */
    ev3_motor_config(PortMotorLeft, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorRight, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorArm1, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorArm2, MEDIUM_MOTOR);
    
    /* Configure sensors */
    ev3_sensor_config(PortUltrasonic, ULTRASONIC_SENSOR);
    ev3_sensor_config(PortSensorColor2, HT_NXT_COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor3, HT_NXT_COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor1, COLOR_SENSOR);


    ev3_lcd_set_font(EV3_FONT_SMALL);

    fprintf(bt, "----GAME_START----\r\n");

    while(ev3_button_is_pressed(ENTER_BUTTON) == false) {}

    /*ここからコーディング */

    /*sensor
    0 = Black, 1 = Purple, 2 =PuBl, 3 = Brue, 4 = Green, 5 = GrYe,
    6 = Yellow, 7 = Orange, 8 = Red, 9 = PiRe, 10 = Pink
, 
    11 = WhBl, 12 = WhGr, 13 = WhYe, 14 = WhOr, 15 = WhRe, 16 = WhPi,
    17 = White*/


    /*スタートの分岐チェック*/
    start = 1;
    tslp_tsk(400*MSEC);
    
    straight_custom(83, 1, 0, -100);

    //marking
    turn(75, 20, -20);
    wall_turn(15, -15);
    steering_time(800, 30, 5);
    tslp_tsk(100*MSEC);
    steering_time(520, -30, 0);
    turn(90, 50, -50);
    steering_time(800, 15, 0);
    tslp_tsk(100*MSEC);
    straight(22, -60, false);
    
    /*
    start_nkc();
    blue_nkc();
    green_nkc();
    yellow_nkc();
    red_nkc();  
    stopping();
    brown_nkc();
    stopping();
    white_nkc();
    stopping();
    chemical_nkc();
    stopping();
    marking_nkc();
    stopping();
    goal_nkc();
    stopping();
    */
}