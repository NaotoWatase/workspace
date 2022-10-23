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


void newsteering(float power, float cm);

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

float distance = 0;
int judgement_check = 0;
int chemical = 0;
int marking_count = 0;

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

void location_perfect(int power, float cm_1st, float cm_2nd, float cm_3rd, float cm_4th, float map_1st, float map_2nd, float map_3rd, float map_4th, float way_1st, float way_2nd, float way_3rd, float way_4th) {
    
    float cm = cm_1st + cm_2nd + cm_3rd + cm_4th; 
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
        if (accele_length * ROBOT1CM > left) {
            steer = p_diff * 6 + p_gyro * 4;
            power = (set_power / accele_length) * (left / ROBOT1CM);
            if (power > -15 && set_power < 0) power = -15;
            if (power < 15 && set_power > 0) power = 15;
            ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
        }
        else if (maxspeed_length * ROBOT1CM > left) {
            steer = p_diff * 5 + p_gyro * 4;
            ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
        }
        else {
            steer = p_diff * 6 + p_gyro * 4;
            power = (-set_power / decele_length) * ((left / ROBOT1CM ) - maxspeed_length) + set_power;
            if (power > -10 && set_power < 0) power = -10;
            if (power < 10 && set_power > 0) power = 10;
            ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
        }
        if (cm * ROBOT1CM <= left) break;
        if (cm_1st*MSEC <= left && way_1st == RIGHT && task_counts == 0) sta_cyc(LOCATION_R_TASK);
        if (cm_1st*MSEC <= left && way_1st == LEFT && task_counts == 0) sta_cyc(LOCATION_L_TASK);
        task_counts = 0;
        if (cm_2nd*MSEC <= left && way_2nd == RIGHT && task_counts == 0) sta_cyc(LOCATION_R_TASK);
        if (cm_2nd*MSEC <= left && way_2nd == LEFT && task_counts == 0) sta_cyc(LOCATION_L_TASK);
        task_counts = 0;
        if (cm_3rd*MSEC <= left && way_3rd == RIGHT && task_counts == 0) sta_cyc(LOCATION_R_TASK);
        if (cm_3rd*MSEC <= left && way_3rd == LEFT && task_counts == 0) sta_cyc(LOCATION_L_TASK);
        task_counts = 0;
        if (cm_4th*MSEC <= left && way_4th == RIGHT && task_counts == 0) sta_cyc(LOCATION_R_TASK);
        if (cm_4th*MSEC <= left && way_4th == LEFT && task_counts == 0) sta_cyc(LOCATION_L_TASK);
        task_counts = 0;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

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
                changing_power = (set_power / (angle*TURN*ROBOT1CM / 4)) * average;
                if (changing_power < 8) changing_power = 8;
                changing_L = changing_power * L_sign;
                changing_R = changing_power * R_sign;
            }
            if (average > angle*TURN*ROBOT1CM * 3 / 4) {
                changing_power = (-set_power / (angle*TURN*ROBOT1CM * 1 / 4)) * (average - (angle*TURN*ROBOT1CM * 3 / 4)) + set_power;
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
}


/*直音が担当します*/
void straight(float cm, float set_power) {
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
    float gein = -0.5;
    float steer;
    float maxspeed_length = cm * 10.0 / 20.0;
    float decele_length = cm * 10.0 / 20.0;
    

    float diff = 0.0006;    // 0.0008 
    if (set_power > 0) {
        diff = 0.0013;
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


void straight_custom(float cm, float ac, float dc, float set_power) {
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
    float gein = -0.5;
    float steer;
    float maxspeed_length = cm * ac;
    float decele_length = cm * dc;
    

    float diff = 0.0006;    // 0.0008 
    if (set_power > 0) {
        diff = 0.0013;
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
        steer =  p * -0.5 + i * 0 + d * 0; 
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
    float distance;
  
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
            distance = ev3_ultrasonic_sensor_get_distance(EV3_PORT_4);
            tslp_tsk(100);
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
            distance = ev3_ultrasonic_sensor_get_distance(EV3_PORT_4);
            tslp_tsk(100);
        }  
    }
    
        red = rgb_val.r;
        green = rgb_val.g;
        blue = rgb_val.b;
        judgement = red + green + blue;
            
        location_sensor[num] = obj;

    if (start == 1 && RIGHT || start == 2 && LEFT){
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
    }
    if (start == 1 && LEFT || start == 2 && RIGHT){
        if (distance < 6) {
            switch (obj){
                case 1:
                case 2:
                case 3:
                    location[num] = ADULT;
                    sensor_check(obj);
                    break;
                case 4:
                case 13:
                    location[num] = CHILD;
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
                    sensor_check(obj);
                    break; 
                default:
                    break;
            } 
        }
        else {
            switch (obj){
                case 1:
                case 2:
                case 3:
                    location[num] = PERSON;
                    sensor_check(obj);
                    break;
                case 4:
                    location[num] = PERSON;
                    sensor_check(obj);
                    break;
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                case 10:
                    location[num] = FIRE;
                    sensor_check(obj);
                    break;
                default:
                    location[num] = NOTHING;
                    break;
            }
        }
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
        steer = (distance - get_distance) * 12;
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
    ev3_sensor_config(PortUltrasonic, ULTRASONIC_SENSOR);
    ev3_sensor_config(PortSensorColor2, HT_NXT_COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor3, HT_NXT_COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor1, COLOR_SENSOR);




    /* Configure sensors */



    ev3_lcd_set_font(EV3_FONT_SMALL);

    fprintf(bt, "----GAME_START----\r\n");

    while(ev3_button_is_pressed(ENTER_BUTTON) == false) {}

    /*ここからコーディング */

    /*sensor
    0 = Black, 1 = Purple, 2 =PuBl, 3 = Brue, 4 = Green, 5 = GrYe,
    6 = Yellow, 7 = Orange, 8 = Red, 9 = PiRe, 10 = Pink, 
    11 = WhBl, 12 = WhGr, 13 = WhYe, 14 = WhOr, 15 = WhRe, 16 = WhPi,
    17 = White*/


    /*スタートの分岐チェック*/


    straight(80, -100);
    turn(90, 20, -20);
    steering_time(1000, -30, 0);
    tslp_tsk(300*MSEC);
    straight(9.5, 50);
    tslp_tsk(300*MSEC);
    //waltrace_length(12, 30, 10);
    turn(90, 20, -20);
    steering_color(COLOR_WHITE, 30, 0);
    steering_color(COLOR_BLACK, 24, 0);
    linetrace_length(26, 7);
    tslp_tsk(400*MSEC);

    /*straight(100, -100);
    tslp_tsk(1000*MSEC);
    steering_time(300, 20, -10);
    tslp_tsk(1000*MSEC);
    turn(90, 20, -20);
    tslp_tsk(1000*MSEC);
    steering_time(600, -20, 0);
    tslp_tsk(1000*MSEC);
    straight(12, 50);
    tslp_tsk(1000*MSEC);
    turn(90, 20, -20);
    tslp_tsk(300*MSEC);
    steering_color(COLOR_WHITE, 25, 0);
    steering_color(COLOR_BLACK, 25, 0);
    linetrace_length(30, 24);

    tslp_tsk(10000*MSEC);*/

    /*
    straight(70, -80);
    turn(180, 50, -50);
    walltrace_length(12, 30, 10);
    steering_color(COLOR_WHITE, 35, 0);
    steering_color(COLOR_BLACK, 35, 0);
    linetrace_length(9, 24);
    */
   
    /* blue */
    straight(8, 20);
    //map_check(0, RIGHT);
    //chemical_taker(0, RIGHT);
    straight(38, 80);
    //map_check(1, RIGHT);
    //chemical_taker(1, RIGHT);
    //water(0);
    //water(1);

    /* green */
    straight(10, 80);
    //map_check(2, RIGHT);
    //chemical_taker(2, RIGHT);
    straight(41, 80);
    //map_check(3, RIGHT);

    steering_time(200, 30, 0);
    if (location[3] == CHEMICAL){
        straight(8, -50);
        turn(90, 50, -50);
        straight(5, 50);
        //chemical_taker(3, LEFT);
        straight(10, -50);
        turn(180, 50, -50);
        steering_time(1000, -30, 0);
    }
    else{
        tslp_tsk(100*MSEC);
        ev3_speaker_play_tone(NOTE_A5, 200);
        tslp_tsk(200*MSEC);
        turn(180, -25, 0);
        ev3_speaker_play_tone(NOTE_A5, 200);
        tslp_tsk(200*MSEC);
        steering_time(800, -25, 0);
        ev3_speaker_play_tone(NOTE_A5, 200);
        tslp_tsk(200*MSEC);
    }
    //water(2);
    //water(3);

    /* yellow */
    straight(25, 80);
    //map_check(4, RIGHT);
    //chemical_taker(4, RIGHT);
    //water(4);

    /* red */
    straight(37.5, 80);
    //map_check(11, RIGHT);
    //chemical_taker(11, RIGHT);
    straight(27, 80);
    //map_check(11, RIGHT);
    //chemical_taker(11, RIGHT);
    steering_time(200, 30, 0);
    ev3_speaker_play_tone(NOTE_A5, 200);
    tslp_tsk(400*MSEC);
    straight(4, -30);
    turn(90, -20, 20);
    ev3_speaker_play_tone(NOTE_A5, 200);
    tslp_tsk(400*MSEC);
    straight(3, -28);
    steering_time(800, -30, 0);
    ev3_speaker_play_tone(NOTE_A5, 200);
    tslp_tsk(200*MSEC);
    //water(10);
    //water(11);

    /* brown */
    straight(34, 80);
    //map_check(9, LEFT);
    //chemical_taker(9, LEFT);
    straight(37, 80);
    tslp_tsk(300*MSEC);
    straight(10, -50);
    turn(90, -25, 25);
    tslp_tsk(300*MSEC);
    straight(3, -28);
    steering_time(1000, -30, 0);
    tslp_tsk(200*MSEC);

    /* white */
    straight(11, 80);
    //map_check(8, RIGHT);
    //water(8);
    //water(9);
    straight(11, 60);
    //map_check(5, RIGHT);
    //map_check(6, LEFT);
    straight(27, 80);
    tslp_tsk(300*MSEC);
    turn(90, 20, -20);
    tslp_tsk(400*MSEC);
    straight(40, -50);
    straight(10, 80);
    straight(38, 80);
    if (location[8] == CHEMICAL){

    }
    if (location[5] == CHEMICAL){

    }
    if (location[6] == CHEMICAL){

    }
    //water(5);
    //water(6);

    /* chemical */
    tslp_tsk(300*MSEC);
    straight(67, 80);
    //関数でいいかも（falseでやりたい）
    /*if (ev3_motor_get_counts(EV3_PORT_D) > 90) {
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
    }*/
    if(chemical_type == RIGHT){
        tslp_tsk(300*MSEC);
        turn(90, -25, 25);
        tslp_tsk(300*MSEC);
        straight(10, -50);
        //ev3_motor_rotate(EV3_PORT_A, 80, -20, true);//数値＆パワーてきとう
        straight(60, -80);
        tslp_tsk(1000*MSEC);
        turn(190, 0, 25);
    }
    else {
        tslp_tsk(300*MSEC);
        turn(90, 25, -25);
        tslp_tsk(300*MSEC);
        straight(10, -50);
        //ev3_motor_rotate(EV3_PORT_A, 80, 20, true);
        straight(70, 80);
        turn(190, 25, 0);
    }
    

    /* crossingB */
    tslp_tsk(200*MSEC);
    straight_custom(90, 1, 0, -100);
    steering_time(2000, -30, -20);
    straight(25, 80);
    turn(90, -25, 25);
    steering_time(800, 30, 0);
    straight(30, -80);

    /* marking */
    if (location[0] == PERSON || location[1] == PERSON) map[0] = 1; //blue
    if (location[2] == PERSON || location[3] == PERSON) map[1] = 1; //green
    if (location[5] == PERSON || location[6] == PERSON) map[2] = 1; //white
    if (location[4] == PERSON || location[7] == PERSON) map[3] = 1; //yellow
    if (location[8] == PERSON || location[9] == PERSON) map[4] = 1; //brown
    if (location[10] == PERSON || location[11] == PERSON) map[5] = 1; //red
    
    if (map[4]) {
        ev3_motor_rotate(EV3_PORT_D, 120, -50, true);//数値＆パワーてきとう
        ev3_motor_rotate(EV3_PORT_D, 120, 50, false);
        straight(22, -50);
        marking_count = 1;
    }
    else{
        ev3_motor_rotate(EV3_PORT_D, 80, -50, true);
        ev3_motor_rotate(EV3_PORT_D, 10, -20, true);
        straight(11, -50);
        if (map[2]) {
            ev3_motor_rotate(EV3_PORT_D, 50, 50, false);
            ev3_motor_rotate(EV3_PORT_D, 80, -50, true);
            ev3_motor_rotate(EV3_PORT_D, 80, 50, false);
            straight(11, -50);
            marking_count = 1;
        }
        else{
            if (map[0]) {
                straight(13, -50);
                steering_time(200, -30, 0);
                ev3_motor_rotate(EV3_PORT_D, 80, -50, true);
                ev3_motor_rotate(EV3_PORT_D, 80, 50, false);
                marking_count = 1;
            }
            else{
                straight(13, -50);
                steering_time(200, 30, 0);
                if (map[1] && marking_count == 0) {
                    ev3_motor_rotate(EV3_PORT_D, 60, -30, true);
                    ev3_motor_rotate(EV3_PORT_D, 60, 50, false);
                    straight(4, -50);
                }
                else if (map[3] && marking_count == 0) {
                    straight(11, 50);
                    ev3_motor_rotate(EV3_PORT_D, 60, -30, true);
                    ev3_motor_rotate(EV3_PORT_D, 60, 50, false);
                    straight(15, -50);
                }
                ev3_motor_rotate(EV3_PORT_D, 40, -30, true);
                ev3_motor_rotate(EV3_PORT_D, 10, -10, false);
            }
        }
    }
    if (map[0] && marking_count == 1) {
        ev3_motor_rotate(EV3_PORT_D, 80, -50, true);
        ev3_motor_rotate(EV3_PORT_D, 80, 50, false);
    }
    if (map[1]) {
        ev3_motor_rotate(EV3_PORT_D, 60, -30, true);
        ev3_motor_rotate(EV3_PORT_D, 60, 50, false);
    }
    if (map[2] && marking_count == 1) {
        ev3_motor_rotate(EV3_PORT_D, 80, -50, true);
        ev3_motor_rotate(EV3_PORT_D, 80, 50, false);
    }
    if (map[3]) {
        ev3_motor_rotate(EV3_PORT_D, 60, -30, true);
        ev3_motor_rotate(EV3_PORT_D, 60, 50, false);
    }
    if (map[5]) {
        ev3_motor_rotate(EV3_PORT_D, 60, -30, true);
        ev3_motor_rotate(EV3_PORT_D, 60, 50, false);
    }

  
    ev3_motor_rotate(EV3_PORT_D, 50, -30, false);
    straight(20, 80);
    turn(90, 0, 25);
    steering_time(1000, 30, -10);


}