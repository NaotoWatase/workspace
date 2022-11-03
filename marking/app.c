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
/* comment added here */
#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

static FILE *bt = NULL;


void newsteering(float power, float cm);

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
static const motor_port_t   PortMotorArm2   = EV3_PORT_D;
static const motor_port_t   PortMotorArm  = EV3_PORT_A;
static const motor_port_t   PortMotorLeft   = EV3_PORT_B;
static const motor_port_t   PortMotorRight  = EV3_PORT_C;




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

float distance = 0;
int judgement_check = 0;
int chemical = 0;


int white = 0;
int how_many = 31;

map_t map [6] = {0,0,0,0,0,0};
int start = 2;

int chemical_type = 0;

int water_count = 0;

int y = 0;


int marking_count = 0;



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

int red=0;
int green=0;
int blue=0;
int judgement = 0;



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


void calculate(int map, int place_saves) {
    savedate_color[place_saves] = obj;
    savedate_red[place_saves] = red;
    savedate_green[place_saves] = green;
    savedate_blue[place_saves] = blue;
    judgement = red + green + blue;
    location_sensor[map] = obj;
    switch (obj){
        case 1:
        case 2:
        case 3:
        case 4:
        case 11:
        case 13:
        case 16:
            location[map] = PERSON;
            break;
        case 12:
        case 14:
        case 17:
            
            if (judgement > 80 || blue > 40 || green > 40 || red > 40) {
                location[map] = PERSON;
            } 
            else {
                judgement_check = judgement;
                location[map] = NOTHING;
            }
            break;
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 15:
            location[map] = FIRE;
            break;
        case 0:
            if (red > 15 && red > blue && red > green) location[map] = FIRE;
            else location[map] = CHEMICAL;
            break; 
        default:
            location[map] = NOTHING; 
    }
    fprintf(bt, "LOCATION = %d\r\nCOLOR = %d  RGB:%d,%d,%d = JUDGE:%d\r\nRESULT = %d\r\n-----------------\r\n", map, obj, red, green, blue, judgement, location[map]);
}

void turn1(float angle, float L_power, float R_power) {
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
    if (L_power == 0) {
        (void)ev3_motor_set_power(EV3_PORT_B, -L_power);
        (void)ev3_motor_rotate(EV3_PORT_B, angle*TURN*ROBOT1CM, (int16_t)-L_power, true);
    } 
    if (L_power == 0) {
        (void)ev3_motor_set_power(EV3_PORT_B, R_power);
        (void)ev3_motor_rotate(EV3_PORT_C, angle*TURN*ROBOT1CM, (int16_t)R_power, true);
    }
    if (L_power != 0 && L_power != 0) {
        set_power = abs(L_power);
        while (true){
            left = ev3_motor_get_counts(EV3_PORT_B);
            right = ev3_motor_get_counts(EV3_PORT_C);
            left = abs(left);
            right = abs(right);
            average = (left + right) / 2.0; 
            if (average < angle*TURN*ROBOT1CM / 2 || set_power > changing_power) {
                changing_power = (set_power / (angle*TURN*ROBOT1CM / 2)) * average;
                if (changing_power < 15) changing_power = 15;
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
        ev3_motor_stop(EV3_PORT_B, true);
    } 
    if (L_power == 0) {
        (void)ev3_motor_rotate(EV3_PORT_C, angle*TURN*ROBOT1CM, (int16_t)R_power, true);
        ev3_motor_stop(EV3_PORT_C, true);
    }
    if (L_power != 0 && R_power != 0) {
        (void)ev3_motor_rotate(EV3_PORT_B, angle*TURN*ROBOT1CM, (int16_t)-L_power, false);
        (void)ev3_motor_rotate(EV3_PORT_C, angle*TURN*ROBOT1CM, (int16_t)R_power, true);
        ev3_motor_stop(EV3_PORT_B, true);
        ev3_motor_stop(EV3_PORT_C, true);
    }
}

/*直音が担当します*/
void straight(float cm, float set_power) {
    float lb_power;
    float rc_power;
    float power = 0;
    float left;
    float right;
    float difference;
    float gein = -0.5;
    float steer;
    float maxspeed_length = cm * 10.0 / 20.0;
    float decele_length = cm * 10.0 / 20.0;

    float diff = 0.0006;    // 0.0008 
    if (set_power > 0) {
        diff = 0.0025;
        gein = -6;
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
    set_power = power;
    while(true){
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        difference = left - right;
        steer = difference * gein;
        power = (-set_power / decele_length) * ((left / ROBOT1CM ) - maxspeed_length) + set_power;
        if (set_power > 0 && power <= 20) {
            power = 20;
        }
        if (set_power < 0 && power >= -20) {
            power = -20;
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
        ev3_motor_rotate(EV3_PORT_D, 80 + water_count, 20, false);
        water_count = water_count + 20;
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
        steer = (distance - get_distance) * 10;
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
        
        p = reflect - 35;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  p * -0.6 + i * 0 + d * 0; 
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
            
            if (judgement > 80 || blue > 40 || green > 40 || red > 40) {
                location[num] = PERSON;
                if (red - green - blue > 20) location[num] = FIRE;
    
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

void chemical_taker(int n, way_t sensor){
    if(location[n] == CHEMICAL){
        chemical = chemical + 1;
       if(sensor == RIGHT){
           ev3_motor_rotate(EV3_PORT_A, 280, -15, false);
           chemical_type = RIGHT;
        }
        else{
            ev3_motor_rotate(EV3_PORT_A, 280, 15, false);
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
    marking_overall(230, 40);
    tslp_tsk(200*MSEC);
    marking_overall(120, -40);
}

void marking_short(){
    marking_overall(220, 15);
    tslp_tsk(200*MSEC);
    marking_overall(120, -40);
}

void main_task(intptr_t unused){

    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    
    /* Register button handlers */
    ev3_button_set_on_clicked(BACK_BUTTON, &button_clicked_handler, BACK_BUTTON);

    /* Configure motors */
    ev3_motor_config(PortMotorLeft, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorRight, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorArm2, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorArm, MEDIUM_MOTOR);
    
    ev3_sensor_config(PortUltrasonic, ULTRASONIC_SENSOR);
    ev3_sensor_config(PortSensorColor1, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor2, HT_NXT_COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor3, HT_NXT_COLOR_SENSOR);



    /* Configure sensors */



    ev3_lcd_set_font(EV3_FONT_SMALL);

    fprintf(bt, "----GAME_START----\r\n");

    while(ev3_button_is_pressed(ENTER_BUTTON) == false) {}

    /*ここからコーディング */
    tslp_tsk(300*MSEC);



    /* marking */
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
    
    
    map[BROWN] = 1;
    map[RED] = 2;


    ev3_motor_reset_counts(EV3_PORT_D);
    steering_time(500, 15, 0);
    straight(11, -50);
    turn(90, -25, 25);
    steering_time(500, 15, 0);
    straight(23, -80);
    //get the first block
    marking_overall(140, 30);
    marking_overall(160, 10);
    //往路
    if (map[BROWN] == 1) marking_long();
    if (map[RED] == 1) marking_short();
    if(map[YELLOW] == 1 || map[WHITE] == 1){
        straight(14.5, -50);
        if (map[WHITE] == 1) marking_long();
        if (map[YELLOW] == 1) marking_short();
        straight(14.5, -50);
    }
    else{
        if(map[BLUE] == 1){
            straight(14, -50);
            turn(30, -20, 0);
            marking_overall(230, 80);
            tslp_tsk(200*MSEC);
            marking_overall(120, -40);
            turn(30, 20, 0);
            straight(15, -50);
        }
        else straight(29, -80);
    }
    marking_overall(160, 10);
    //復路
    if (map[GREEN] == 2) marking_short();
    if (map[BLUE] == 2) marking_long();
    if(map[YELLOW] == 2 || map[WHITE] == 2){
        straight(14.5, 50);
        if (map[YELLOW] == 2) marking_short();
        if (map[WHITE] == 2) marking_long();
        straight(14.5, 50);
    }
    else{
        straight(29, 80);
        if(map[RED] == 2) marking_short();
    }

    marking_overall(180, 30);
    straight(11, 80);
    turn(190, 25, 0);
    steering_time(1000, -30, 5);

}