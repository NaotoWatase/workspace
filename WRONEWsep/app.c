/**
 * This sample program shows a PID controller for line following.
 *
 * Robot construction: Educator Vehicle
 *
 * References:
 * http://robotsquare.com/wp-content/uploads/2013/10/45544_educator.pdf
 * http://thetechnicgear.com/2014/03/howto-create-line-following-robot-using-mindstorms/
 */

#define _APP_C_

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

#define EXTERN

static FILE *bt = NULL;

/**
 * Define the connection ports of the sensors and motors.
 * By default, this application uses the following ports:
 * Touch sensor: Port 2
 * Color sensor: Port 3
 * Left motor:   Port B
 * Right motor:  Port C
 */
/*static const sensor_port_t  PortUltrasonic = EV3_PORT_1;
static const sensor_port_t  PortSensorColor2 = EV3_PORT_2;
static const sensor_port_t  PortSensorColor3 = EV3_PORT_3;
static const sensor_port_t  PortSensorColor4 = EV3_PORT_4;
static const motor_port_t   PortMotorWater   = EV3_PORT_D;
static const motor_port_t   PortMotorLeft   = EV3_PORT_B;
static const motor_port_t   PortMotorRight  = EV3_PORT_C;
static const motor_port_t   PortMotorArm  = EV3_PORT_A;*/
static const motor_port_t   PortMotorLeft   = EV3_PORT_B;
static const motor_port_t   PortMotorRight  = EV3_PORT_C;
static const motor_port_t   PortMotorArm1  = EV3_PORT_A;
static const motor_port_t   PortMotorArm2  = EV3_PORT_D;
static const sensor_port_t PortSensorGYRO = EV3_PORT_4;
static const sensor_port_t PortSensorColor1 = EV3_PORT_1;
static const sensor_port_t PortSensorColor2 = EV3_PORT_2;
static const sensor_port_t PortSensorColor3 = EV3_PORT_3;



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

int kakudo_C = 0; 
int power = 50;

float distance = 0;
int judgement_check = 0;
int chemical = 0;


int white = 0;
int how_many = 31;

int map [6] = {0,0,0,0,0,0};
int start = 2;

int chemical_type = 0;

int water_count = 0;

int y = 0;





uint8_t obj = 0;
uint8_t test = 0;
rgb_raw_t testrgb;

int location[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0};

int location_sensor[12] = {0,0,0,0,0,0,0,0,0,0,0,0};


rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)

int red=0;
int green=0;
int blue=0;
int judgement = 0;


void map_check(int num, way_t sensor) {
    /*ev3_speaker_play_tone(NOTE_A5, 100);
    tslp_tsk(300*MSEC);*/
    
    if(sensor == RIGHT){
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
        tslp_tsk(100);
        
    } 
    else{ 
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
    }
    
    red = rgb_val.r;
    green = rgb_val.g;
    blue = rgb_val.b;
    judgement = red + green + blue;
    
    location_sensor[num] = obj;

    switch (obj){
        case 1:
        case 2:
        case 3:
        case 4:
        case 11:
        case 13:
        case 16:
            location[num] = PERSON;
            break;
        case 12:
        case 14:
        case 17:
            
            if (judgement > 80 || blue > 40 || green > 40 || red > 40) {
                location[num] = PERSON;
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
    fprintf(bt, "LOCATION = %d\r\nCOLOR = %d  RGB:%d,%d,%d = JUDGE:%d\r\nRESULT = %d\r\n-----------------\r\n", num, obj, red, green, blue, judgement, location[num]);
    /*ev3_speaker_play_tone(NOTE_A5, 100);
    tslp_tsk(400*MSEC);*/
}

void chemical_taker(int n, way_t sensor){
    if(location[n] == CHEMICAL){
        chemical = chemical + 1;
       if(sensor == RIGHT){
           ev3_motor_rotate(EV3_PORT_A, 280, -20, false);
           chemical_type = RIGHT;
        }
        else{
            ev3_motor_rotate(EV3_PORT_A, 280, 20, false);
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

void water(int n) {
    if (location[n] == FIRE) {
        ev3_motor_rotate(EV3_PORT_D, 80 + water_count, 20, false);
        water_count = water_count + 20;
    }
}


void sensor_check(uint8_t num) {
    int ct = 0;
    while (ct < num) {
        ev3_speaker_play_tone(NOTE_C5, 100);
        tslp_tsk(500*1000);
        ct = ct + 1;
    }
}

void check_task(intptr_t unused){
    fprintf(bt, "\r\ny:%d", y);
}

void main_task(intptr_t unused){

    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    
    /* Register button handlers */
    ev3_button_set_on_clicked(BACK_BUTTON, &button_clicked_handler, BACK_BUTTON);

    /* Configure motors */
    ev3_motor_config(PortMotorLeft, LARGE_MOTOR);
    ev3_motor_config(PortMotorRight, LARGE_MOTOR);
    ev3_motor_config(PortMotorArm1, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorArm2, MEDIUM_MOTOR);



    /* Configure sensors */

    ev3_sensor_config(PortSensorGYRO, GYRO_SENSOR);
    ev3_sensor_config(PortSensorColor1, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor2, HT_NXT_COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor3, HT_NXT_COLOR_SENSOR);


    ev3_lcd_set_font(EV3_FONT_SMALL);

    while ( !ht_nxt_color_sensor_measure_color(EV3_PORT_3, &test) ) {
            ;
        }
    while ( !ht_nxt_color_sensor_measure_color(EV3_PORT_2, &test) ) {
            ;
        }   
    ev3_color_sensor_get_color(EV3_PORT_1);

    fprintf(bt, "----GAME_START----\r\n");

    while(ev3_button_is_pressed(ENTER_BUTTON) == false) {}

    /*ここからコーディング */

    /*sensor
    0 = Black, 1 = Purple, 2 =PuBl, 3 = Brue, 4 = Green, 5 = GrYe,
    6 = Yellow, 7 = Orange, 8 = Red, 9 = PiRe, 10 = Pink, 
    11 = WhBl, 12 = WhGr, 13 = WhYe, 14 = WhOr, 15 = WhRe, 16 = WhPi,
    17 = White*/


    /*スタートの分岐チェック*/
   
   
   
    start = 1;
    

    switch (start){
    case 1:




    

    newsteering(-90, 80);
    tslp_tsk(100*MSEC);
    steering_time(600, 30, 0);
    newsteering(-50, 11);
    tslp_tsk(200*MSEC);
    p_turn(100, 0, 1);
    tslp_tsk(200*MSEC);
    p_turn(80, -1, 1);
    steering_color(COLOR_WHITE, 35, 0);
    steering_color(COLOR_BLACK, 35, 0);
    steering_time(700, 15, 0);
    linetrace_length(24, 11);
    /*steering_time(900, -25, 0);
    tslp_tsk(200*MSEC);
    newsteering(20, 4.3);
    tslp_tsk(500*MSEC);
    p_turn(88, 0, 1);
    newsteering(25, 9);
    tslp_tsk(500*MSEC);*/
    newsteering(20, 4);
    tslp_tsk(500*MSEC);
    
    ev3_motor_reset_counts(EV3_PORT_D);
    
    map_check(8, RIGHT);
    chemical_taker(8, RIGHT);
    tslp_tsk(600*MSEC);
    newsteering(70, 36);
    map_check(9, RIGHT);
    chemical_taker(9, RIGHT);
    water(8);
    water(9);
    newsteering(75, 48);
    map_check(10, RIGHT);
    map_check(11, LEFT);
    steering_time(500, 30, 0);
    

    /*int cm = 84;
    int set_power = -power;
    int p;
    int gyro;
    int left;
    int right;
    int difference;
    float accele_length = cm / 10 * 1;
    float maxspeed_length = cm / 10 * 9;
    float decele_length = cm;
    int count1 = 0;
    int count2 = 0; 
    
    ev3_gyro_sensor_reset(EV3_PORT_4);
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);	
    while(true){
        gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        difference = left - right;
        p = -(difference + gyro);
        steer = p * 3;
        power = (set_power / accele_length) * (left / ROBOT1CM);
        if (power > -20 && set_power < 0) power = -20;
        if (power < 20 && set_power > 0) power = 20;
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
        if (accele_length * ROBOT1CM <= left) break;
    }
    while(true){
        gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        difference = left - right;
        p = -(difference + gyro);
        steer = p * 3;
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
        if (37 * ROBOT1CM <= left && count1 == 0){
            count1 = count1 + 1; 
            map_check(9, RIGHT);
            chemical_taker(9, RIGHT);
            water(9);
            water(8);
        }
        if (maxspeed_length * ROBOT1CM <= left) break;
    }
    while(true){
        gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        difference = left - right;
        p = -(difference + gyro);
        steer = p * 3;
        power = (-set_power / accele_length) * ((left / ROBOT1CM ) - maxspeed_length) + set_power;

        if (power > -20 && set_power < 0) power = -20;
        if (power < 20 && set_power > 0) power = 20;
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
        
        if (decele_length * ROBOT1CM <= left) break;
    }

    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);

    tslp_tsk(1000*MSEC);
    map_check(10, RIGHT);
    tslp_tsk(1000*MSEC);
    map_check(11, LEFT);

    tslp_tsk(500*MSEC);
    ev3_speaker_play_tone(NOTE_A6, 200);
    tslp_tsk(500*MSEC);
    sensor_check(location[8]);
    tslp_tsk(500*MSEC);
    ev3_speaker_play_tone(NOTE_A6, 200);
    tslp_tsk(500*MSEC);
    sensor_check(location[9]);
    tslp_tsk(500*MSEC);
    ev3_speaker_play_tone(NOTE_A6, 200);
    tslp_tsk(500*MSEC);
    sensor_check(location[10]);
    tslp_tsk(500*MSEC);
    ev3_speaker_play_tone(NOTE_A6, 200);
    tslp_tsk(500*MSEC);
    sensor_check(location[11]);
    tslp_tsk(500*MSEC);
    ev3_speaker_play_tone(NOTE_A6, 200);
    tslp_tsk(500*MSEC);
    sensor_check(judgement_check);*/

    

    if (location[10] == CHEMICAL){
        ev3_motor_rotate(EV3_PORT_A, 280, -20, false);
        newsteering(-30, 6);
        p_turn(90, 1, -1);
        steering_time(200, 20, 0);
        chemical_taker(10, LEFT);
        tslp_tsk(1000*MSEC);
        p_turn(90, -1, 1);
        chemical_took(10, LEFT);
        steering_time(800, 25, 0);
    }
    if (location[11] == CHEMICAL){
        newsteering(-30, 6);
        p_turn(88, -1, 1);
        chemical_taker(11, RIGHT);
        tslp_tsk(700*MSEC);
        p_turn(88, 1, -1);
        chemical_took(11, RIGHT);
        steering_time(800, 25, 0);        
    }
    
    newsteering(-50, 14);
    p_turn(90, 1, -1);
    if(chemical < 1)ev3_motor_rotate(EV3_PORT_A, 280, -20, false);
    newsteering(-50, 10);
    steering_time(500, -30, 0);
    water(10);
    water(11);
    newsteering(70, 23);
    map_check(7, RIGHT);
    if (location[7] == CHEMICAL){
        ev3_motor_rotate(EV3_PORT_A, 280, 20, false);
        newsteering(80, 26);
        tslp_tsk(400*MSEC);
        p_turn(90, 1, -1);
        steering_time(200, 25, 0);
        chemical_taker(7, RIGHT);
        tslp_tsk(1000*MSEC);
        p_turn(90, -1, 1);
        newsteering(-50, 14);
        tslp_tsk(200*MSEC);
        p_turn(60, 0, 1);
        tslp_tsk(300*MSEC);
        p_turn(60, 1, 0);
    }
    else{
        p_turn(58, 0, 1);
        tslp_tsk(300*MSEC);
        p_turn(58, 1, 0);
        newsteering(45, 10);
    }
    map_check(4, LEFT);
    chemical_taker(4, LEFT);
    water(4);
    water(7);
    newsteering(70, 38);
    map_check(3, LEFT);
    chemical_taker(3, LEFT);
    steering_time(1000, 30, 0);
    tslp_tsk(200*MSEC);
    tank_turn(80, 0, -30);
    tslp_tsk(200*MSEC);
    p_turn(50, 1, -1);
    steering_time(1100, -25, 0);

    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    tslp_tsk(200*MSEC);
    newsteering(60, 21.5);
    map_check(2, LEFT);
    chemical_taker(2, LEFT);
    water(2);
    water(3);
    newsteering(50, 10.2);
    map_check(1, LEFT);
    chemical_taker(1, LEFT);
    newsteering(65, 36.8);
    map_check(0, LEFT);
    chemical_taker(0, LEFT);
    water(0);
    water(1);

    white = how_many - (location[0] + location[1] + location[2] + location[3] + location[4] + location[7] + location[8] + location[9] + location[10] + location[11]);

    if (white == CHEMICAL){
        newsteering(-35, 6);
        tslp_tsk(500*MSEC);
        p_turn(90, 1, -1);
        steering_time(800, -30, 0);
        newsteering(70, 44);
        tslp_tsk(500*MSEC);
        p_turn(90, 1, -1);
        newsteering(40, 5);
        map_check(6, LEFT);
        chemical_taker(6, LEFT);
        p_turn(180, 1, -1);
        newsteering(60, 10);
        map_check(5, RIGHT);
        chemical_taker(5, RIGHT);
        newsteering(80, 63);
        tslp_tsk(200*MSEC);
        p_turn(90, 1, -1);
        newsteering(-40, 16);
    }    
    else if (white == FIRE){
        location[5] = FIRE;
        newsteering(-30, 6);
        p_turn(90, -1, 1);
        newsteering(-70, 44);
        p_turn(90, 1, -1);
        water(5);
        tslp_tsk(700*MSEC);
        newsteering(80, 70);
        tslp_tsk(200*MSEC);
        p_turn(90, 1, -1);
    }
    else{
    newsteering(80, 66);
    tslp_tsk(200*MSEC);
    p_turn(90, 1, -1);
    steering_time(1500, -30, 0);
    newsteering(70, 40);
    }
    if(chemical_type == RIGHT){
        newsteering(50, 24);
        p_turn(180, 1, -1);
        ev3_motor_rotate(EV3_PORT_A, 80, 20, true);
        ev3_motor_rotate(EV3_PORT_A, 185, 12, true);
        ev3_motor_rotate(EV3_PORT_A, 30, 15, false);
        tslp_tsk(500 * MSEC);
        p_turn(180, -1, 1);
        
    }
    else{
        ev3_motor_rotate(EV3_PORT_A, 80, -20, true);
        ev3_motor_rotate(EV3_PORT_A, 185, -12, true);
        ev3_motor_rotate(EV3_PORT_A, 30, -15, false);
        tslp_tsk(500 * MSEC);
    }
    
    if (ev3_motor_get_counts(EV3_PORT_D) > 90) {
        while(true){
            ev3_motor_set_power(EV3_PORT_D, -20);
            if (ev3_motor_get_counts(EV3_PORT_D) < 90)break;
        }
        ev3_motor_stop(EV3_PORT_D, true);
    }
    if (ev3_motor_get_counts(EV3_PORT_D) < 90) {
        while(true){
            ev3_motor_set_power(EV3_PORT_D, 20);
            if (ev3_motor_get_counts(EV3_PORT_D) > 90)break;
        }
        ev3_motor_stop(EV3_PORT_D, true);
    }
    ev3_motor_stop(EV3_PORT_D, true);
    
    if(chemical_type == LEFT)newsteering(-70, 46);
    if(chemical_type == RIGHT)newsteering(-70, 66);

    steering_time(700, -30, 0);
    tslp_tsk(200*MSEC);

    steering_time(200, 30, 0);
    tank_turn(80, 30, 0);
    tslp_tsk(200*MSEC);
    tank_turn(98, 0, -30);
    steering(-65, 80, 0);
    steering_time(1800, -30, 0);
    



    


    newsteering(50, 17);
    tank_turn(140, 0, 45);
    tslp_tsk(200*MSEC);
    tank_turn(160, 45, -45);
    steering_time(1000, 25, 0);
    

    newsteering(-70, 65);
    ev3_motor_rotate(EV3_PORT_D, 90, 30, true);
    tslp_tsk(200*MSEC);
    tank_turn(185, -30, 0);
    tslp_tsk(400*MSEC);
    tank_turn(184, 0, -30);
    tslp_tsk(200*MSEC);
    newsteering(70, 23);
    
    
    if (location[0] == PERSON || location[1] == PERSON) map[0] = 1; /*blue*/
    if (location[2] == PERSON || location[3] == PERSON) map[1] = 1; /*green*/
    if (location[5] == PERSON || location[6] == PERSON) map[2] = 1; /*white*/
    if (location[4] == PERSON || location[7] == PERSON) map[3] = 1; /*yellow*/
    if (location[8] == PERSON || location[9] == PERSON) map[4] = 1; /*brown*/
    if (location[10] == PERSON || location[11] == PERSON) map[5] = 1; /*red*/
   
    
    if (map[4] == 1 || map[5] == 1) {
        if (map[4] == 1) {
            ev3_motor_rotate(EV3_PORT_D, 30, 40, true); /*時計回りがプラス*/
            newsteering(30, 3);
            ev3_motor_rotate(EV3_PORT_D, 40, -10, true);
            ev3_motor_rotate(EV3_PORT_D, 10, 40, true);
            newsteering(30, 3.5);
        }
        else newsteering(50, 6.5);
        if (map[5] == 1) {
            ev3_motor_rotate(EV3_PORT_D, 30, -40, true); 
            newsteering(30, 3);
            ev3_motor_rotate(EV3_PORT_D, 40, 25, true);
            ev3_motor_rotate(EV3_PORT_D, 10, -40, true);
            newsteering(30, 5);
        }
        else newsteering(50, 8);
    }
    else newsteering(50, 14.5);

    if (map[2] == 1 || map[3] == 1) {
        if (map[2] == 1) {
            ev3_motor_rotate(EV3_PORT_D, 30, 40, true); 
            newsteering(30, 3);
            ev3_motor_rotate(EV3_PORT_D, 40, -10, true);
            ev3_motor_rotate(EV3_PORT_D, 10, 40, true);
            newsteering(30, 3.5);
        }
        else newsteering(50, 6.5);
        if (map[3] == 1) {
            ev3_motor_rotate(EV3_PORT_D, 30, -40, true); 
            newsteering(30, 3);
            ev3_motor_rotate(EV3_PORT_D, 40, 25, true);
            ev3_motor_rotate(EV3_PORT_D, 10, -40, true);
            newsteering(30, 5);
        }
        else newsteering(50, 8);
    }
    else newsteering(50, 14.5);
    
    if (map[0] == 1 || map[1] == 1) {
        if (map[0] == 1) {
            ev3_motor_rotate(EV3_PORT_D, 30, 40, true); 
            newsteering(30, 3);
            ev3_motor_rotate(EV3_PORT_D, 40, -10, true);
            ev3_motor_rotate(EV3_PORT_D, 10, 40, true);
            newsteering(30, 3.5);
        }
        else newsteering(50, 6.5);
        if (map[1] == 1) {
            ev3_motor_rotate(EV3_PORT_D, 30, -40, true); 
            newsteering(30, 3);
            ev3_motor_rotate(EV3_PORT_D, 40, 25, true);
            ev3_motor_rotate(EV3_PORT_D, 10, -40, true);
        }
    }
    steering_time(3000, 30, -40);
    
    break;
    
    case 2:
        break;
    }

    


    

}