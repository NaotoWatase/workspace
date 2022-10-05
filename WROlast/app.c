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

#define MSEC (1000)
#define ROBOT1CM (18.48)
#define TURN (0.162)

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




/*mapのデータ変数*/
typedef enum object {  
    PERSON = 10,
    CHILD = 3,
    ADULT = 2,
    FIRE = 5,
    CHEMICAL = 1,
    NOTHING = 0
} object_t ;

/*mapceck時のセンサーを指定*/
typedef enum way{
    RIGHT,
    LEFT
} way_t ;

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

void steering(int power, int cm, int steering) {
    int b_counts;
    int c_counts;
    cm = abs(cm);
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    power = -power;
    ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steering);
    while (true) {
        b_counts = abs(ev3_motor_get_counts(EV3_PORT_B));
        c_counts = abs(ev3_motor_get_counts(EV3_PORT_C));
        if (cm * ROBOT1CM < b_counts || cm * ROBOT1CM < c_counts) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

/*
void newsteering(int power, float cm) {
    int set_power = -power;
    int p;
    int gyro;
    float steer;
    int left;
    int right;
    int difference;
    float accele_length = cm / 10 * 0;
    float maxspeed_length = cm / 10 * 10;
    float decele_length = cm;

    int left_diff = 0;
    int left_last = 0;
    int right_diff = 0;
    int right_last = 0;
    int average = 0;

    
    
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
        left_last = left;
        left = ev3_motor_get_counts(EV3_PORT_B);
        left_diff = left - left_last;

        right_last = right;
        right = ev3_motor_get_counts(EV3_PORT_C);
        right_diff = right - right_last;

        average = (left_diff + right_diff) / 2;

        difference = left - right;
        float diff = sin(gyro) * average;
        y = y + diff;
        float coef_diff = 0.0;
        float coef_gyro = 0;
        p = -(gyro * 5 + y * coef_gyro);
        steer = p;
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, set_power, steer);
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
    ev3_speaker_play_tone(NOTE_A4, 200);
    tslp_tsk(1000);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}*/

void newsteering(int power, float cm) {
    int set_power = -power;
    int p_gyro;
    int p_diff;
    int gyro;
    int left;
    int right;
    int difference;
    float accele_length = cm / 10 * 1;
    float maxspeed_length = cm / 10 * 9;
    float decele_length = cm / 10 * 1;
    float steer;
    
    
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
        if (set_power > 0) p_gyro = gyro;
        else p_gyro = -gyro;
        p_diff = -difference;
        steer = p_diff * 6 + p_gyro * 4;
        power = (set_power / accele_length) * (left / ROBOT1CM);
        if (power > -15 && set_power < 0) power = -15;
        if (power < 15 && set_power > 0) power = 15;
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
        if (set_power > 0) p_gyro = gyro;
        else p_gyro = -gyro;
        p_diff = -difference;
        steer = p_diff * 5 + p_gyro * 4;
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
        if (maxspeed_length * ROBOT1CM <= left) break;
    }
    while(true){
        gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        difference = left - right;
        if (set_power > 0) p_gyro = gyro;
        else p_gyro = -gyro;
        p_diff = -difference;
        steer = p_diff * 6 + p_gyro * 4;
        power = (-set_power / decele_length) * ((left / ROBOT1CM ) - maxspeed_length) + set_power;
        /*if (power < 20 && power >= 0 && set_power > 0) power = 20;
        if (power > -20 && power <= 0 && set_power < 0) power = -20;*/
        if (power > -10 && set_power < 0) power = -10;
        if (power < 10 && set_power > 0) power = 10;
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
        if (cm * ROBOT1CM <= left) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    
}


void steering_time(int time_stop_4d, int power, int steering){
    power = -power;
    if(steering > 0) {
    (void)ev3_motor_set_power(EV3_PORT_B, power);
    (void)ev3_motor_set_power(EV3_PORT_C, power+(power*steering/50));
    }
    else {
    (void)ev3_motor_set_power(EV3_PORT_B, power-(power*steering/50));
    (void)ev3_motor_set_power(EV3_PORT_C, power);
    }
    tslp_tsk(time_stop_4d * MSEC);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}


void steering_color(colorid_t color_stop, int power, int steering){
    colorid_t color;
    power = -power;
    if(steering > 0) {
    (void)ev3_motor_set_power(EV3_PORT_B, power);
    (void)ev3_motor_set_power(EV3_PORT_C, power+(power*steering/50));
    }
    else {
    (void)ev3_motor_set_power(EV3_PORT_B, power-(power*steering/50));
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
    float maxspeed_length = angle / 10 * 3;
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
    tslp_tsk(200*MSEC);
}

void wait(float cm) {
    int save;
    int cm_counts;
    cm_counts = abs(ev3_motor_get_counts(EV3_PORT_B));
    save = cm_counts;
    while (true){
        cm_counts = abs(ev3_motor_get_counts(EV3_PORT_B));
        if (cm_counts >= cm*ROBOT1CM + save) break;
    }
}
    
void tank_turn(float angle, int power_L, int power_R){
    if (power_R == 0) {
        (void)ev3_motor_rotate(EV3_PORT_B, angle*TURN*ROBOT1CM, (int16_t)-power_L, true);
    } 
    if (power_L == 0) {
        (void)ev3_motor_rotate(EV3_PORT_C, angle*TURN*ROBOT1CM, (int16_t)-power_R, true);
    }
    if (power_L != 0 && power_R != 0) {
        (void)ev3_motor_rotate(EV3_PORT_B, angle*TURN*ROBOT1CM, (int16_t)-power_L, false);
        (void)ev3_motor_rotate(EV3_PORT_C, angle*TURN*ROBOT1CM, (int16_t)-power_R, true);
        ev3_motor_stop(EV3_PORT_B, true);
        ev3_motor_stop(EV3_PORT_C, true);
    }
}


void linetrace_length(float length, int power){
    ev3_motor_reset_counts(EV3_PORT_C);
    float steer;
    float p = 0;
    float i = 0;
    float d = 0;
    float d2 = 0;
    int reflect;
    power = -power;
    while (true) {
        kakudo_C = ev3_motor_get_counts(EV3_PORT_C);
        reflect = ev3_color_sensor_get_reflect(EV3_PORT_1);
        
        p = reflect - 35;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  -p * P_GEIN + i * 0 + d * 0; 
        if (length * ROBOT1CM < -kakudo_C) break;
        if(steer > 0) {
        (void)ev3_motor_set_power(EV3_PORT_B, power);
        (void)ev3_motor_set_power(EV3_PORT_C, power+(power*steer/50));
        }
        else {
        (void)ev3_motor_set_power(EV3_PORT_B, power-(power*steer/50));
        (void)ev3_motor_set_power(EV3_PORT_C, power);
        }

    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
}


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
            location[num] = CHEMICAL;
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



    
    map[0] = 0; /*blue*/
    map[1] = 1; /*green*/
    map[2] = 0; /*white*/
    map[3] = 1; /*yellow*/
    map[4] = 0; /*brown*/
    map[5] = 0; /*red*/
    
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    ev3_motor_set_power(EV3_PORT_B, -18);
    ev3_motor_set_power(EV3_PORT_C, -18);
    if (map[4] == 1 || map[5] == 1) {
        if (map[4] == 1) {
            ev3_motor_rotate(EV3_PORT_D, 30, 40, false); /*時計回りがプラス*/
            wait(2);
            ev3_motor_rotate(EV3_PORT_D, 40, -10, true);
            ev3_motor_rotate(EV3_PORT_D, 10, 40, false);
            wait(4.5);
        }
        else wait(6.5);
        if (map[5] == 1) {
            ev3_motor_rotate(EV3_PORT_D, 30, -40, false); 
            wait(2.3);
            ev3_motor_rotate(EV3_PORT_D, 40, 20, true);
            ev3_motor_rotate(EV3_PORT_D, 10, -40, false);
            wait(5.7);
        }
        else wait(8);
    }
    else wait(14.5);

    if (map[2] == 1 || map[3] == 1) {
        if (map[2] == 1) {
            ev3_motor_rotate(EV3_PORT_D, 30, 40, false); 
            wait(2);
            ev3_motor_rotate(EV3_PORT_D, 40, -10, true);
            ev3_motor_rotate(EV3_PORT_D, 10, 40, false);
            wait(4.5);
        }
        else wait(6.5);
        if (map[3] == 1) {
            ev3_motor_rotate(EV3_PORT_D, 30, -40, false); 
            wait(2.3);
            ev3_motor_rotate(EV3_PORT_D, 40, 20, true);
            ev3_motor_rotate(EV3_PORT_D, 10, -40, false);
            wait(5.7);
        }
        else wait(8);
    }
    else wait(14.5);
    
    if (map[0] == 1 || map[1] == 1) {
        if (map[0] == 1) {
            ev3_motor_rotate(EV3_PORT_D, 30, 40, false); 
            wait(2);
            ev3_motor_rotate(EV3_PORT_D, 40, -10, true);
            ev3_motor_rotate(EV3_PORT_D, 10, 40, false);
            wait(4.5);
        }
        else wait(6.5);
        if (map[1] == 1) {
            ev3_motor_rotate(EV3_PORT_D, 30, -40, false); 
            wait(2.3);
            ev3_motor_rotate(EV3_PORT_D, 40, 20, true);
            ev3_motor_rotate(EV3_PORT_D, 10, -40, false);
            wait(5.7);
        }
        else wait(8);
    }
    else wait(14.5);
    steering_time(2000, 30, -40);
    
    break;
    
    case 2:
        break;
    }

    


    

}