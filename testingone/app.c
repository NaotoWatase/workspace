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
void hsv(int num, way_t sensor);

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
/*static const motor_port_t   PortMotorWater   = EV3_PORT_D;
static const motor_port_t   PortMotorLeft   = EV3_PORT_B;
static const motor_port_t   PortMotorRight  = EV3_PORT_C;
static const motor_port_t   PortMotorArm  = EV3_PORT_A;*/
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


float h = 0;
float s = 0;
float v = 0;
float min = 0;
float max = 0;
float red = 0;
float green = 0;
float blue = 0;
float judgement = 0;
float distance = 0;

int check_type;



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


void newsteering(float power, float cm) {
    float set_power = -power;
    float p_diff;
    float left;
    float right;
    float difference;
    float maxspeed_length = cm * 17.0 / 20.0;
    float decele_length = cm * 3.0 / 20.0;
    float steer;
    float diff = -0.0008;    // 0.0008 
    //ev3_gyro_sensor_reset(EV3_PORT_4);
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);	
    power = diff;
    while(power > set_power){
        //gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        difference = left - right;
        //if (set_power > 0) p_gyro = gyro;
        //else p_gyro = -gyro;
        p_diff = -difference;
        steer = p_diff * 2;
//        if (power > -10 && set_power < 0) power = -10;
//        if (power < 10 && set_power > 0) power = 10;
//        power = (set_power / accele_length) * (left / ROBOT1CM);
        power += diff; 
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
//        if (accele_length * ROBOT1CM <= left) break;
    }
    while(true){
        //gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        difference = left - right;
        //if (set_power > 0) p_gyro = gyro;
        //else p_gyro = -gyro;
        p_diff = -difference;
        steer = p_diff * 2;
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, set_power, steer);
        if (maxspeed_length * ROBOT1CM <= left) break;
    }
    while(true){
        //gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        difference = left - right;
        //if (set_power > 0) p_gyro = gyro;
        //else p_gyro = -gyro;
        p_diff = -difference;
        steer = p_diff * 2;
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

void m_steering(float power, float cm) {
    float set_power = -power;
    float p_diff;
    float left;
    float right;
    float difference;
    float maxspeed_length = cm * 17.0 / 20.0;
    float decele_length = cm * 3.0 / 20.0;
    float steer;
    float diff = -0.0008;    // 0.0008 
    //ev3_gyro_sensor_reset(EV3_PORT_4);
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);	
    power = diff;
    while(power > set_power){
        //gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        difference = left - right;
        //if (set_power > 0) p_gyro = gyro;
        //else p_gyro = -gyro;
        p_diff = -difference;
        steer = p_diff * 2;
//        if (power > -10 && set_power < 0) power = -10;
//        if (power < 10 && set_power > 0) power = 10;
//        power = (set_power / accele_length) * (left / ROBOT1CM);
        power += diff; 
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, 0);
//        if (accele_length * ROBOT1CM <= left) break;
    }
    while(true){
        //gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        difference = left - right;
        //if (set_power > 0) p_gyro = gyro;
        //else p_gyro = -gyro;
        p_diff = -difference;
        steer = p_diff * 2;
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, set_power, steer);
        if (maxspeed_length * ROBOT1CM <= left) break;
    }
    while(true){
        //gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        difference = left - right;
        //if (set_power > 0) p_gyro = gyro;
        //else p_gyro = -gyro;
        p_diff = -difference;
        steer = p_diff * 2;
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

void straight(float set_power, float cm) {
    float lb_power;
    float rc_power;
    float power = 0;
    float left;
    float right;
    float maxspeed_length = cm * 15.0 / 20.0;
    float decele_length = cm * 5.0 / 20.0;

    float diff = 0.0006;    // 0.0008 
    if (set_power > 0) diff = 0.003;
    if (set_power < 0) diff = -0.003;
    //ev3_gyro_sensor_reset(EV3_PORT_4);
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);	
    power = diff;
    while(true){
        //gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        //if (set_power > 0) p_gyro = gyro;
        //else p_gyro = -gyro;
//        if (power > -10 && set_power < 0) power = -10;
//        if (power < 10 && set_power > 0) power = 10;
//        power = (set_power / accele_length) * (left / ROBOT1CM);
        power += diff; 
        lb_power = -power;
        rc_power = power;
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (power >= set_power && set_power > 0) break;
        if (power <= set_power && set_power < 0) break;
        if (left >= maxspeed_length * ROBOT1CM) break;
//        if (accele_length * ROBOT1CM <= left) break;
    }
    while(true){
        //gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        //if (set_power > 0) p_gyro = gyro;
        //else p_gyro = -gyro;
        lb_power = -set_power;
        rc_power = set_power;
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (maxspeed_length * ROBOT1CM <= left) break;
    }
    while(true){
        //gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        //if (set_power > 0) p_gyro = gyro;
        //else p_gyro = -gyro;
        power = (-set_power / decele_length) * ((left / ROBOT1CM ) - maxspeed_length) + set_power;
        /*if (power < 20 && power >= 0 && set_power > 0) power = 20;
        if (power > -20 && power <= 0 && set_power < 0) power = -20;*/
        if (power > -10 && set_power < 0) power = -10;
        if (power < 10 && set_power > 0) power = 10;
        lb_power = -power;
        rc_power = power;
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
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

void newsteering_test(float power, float cm, float steer) {
    float lb_power;
    float rc_power;

    
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);	
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
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
    tslp_tsk(800*MSEC);
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

void turn(int angle, int left_motor, int right_motor){
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
    float steer;
    float p = 0;
    float i = 0;
    float d = 0;
    float d2 = 0;
    int reflect;
    power = -power;
    while (true) {
        kakudo_C = ev3_motor_get_counts(EV3_PORT_C);
        kakudo_B = ev3_motor_get_counts(EV3_PORT_B);
        reflect = ev3_color_sensor_get_reflect(EV3_PORT_1);
        
        p = reflect - 35;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  -p * P_GEIN + i * 0 + d * 0; 
        if (length * ROBOT1CM < -(kakudo_C + kakudo_B) / 2) break;
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
         
        location_sensor[num] = obj;

    if (start == 1 && sensor == RIGHT || start == 2 && sensor == LEFT){
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
        if (distance < 7) {
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

void obj_check(int num, way_t sensor){
    obj_measure(num, sensor);
    obj_know(num);
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
            distance = ev3_ultrasonic_sensor_get_distance(EV3_PORT_4);
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
            distance = ev3_ultrasonic_sensor_get_distance(EV3_PORT_4);
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

    fprintf(bt, "LOCATION = %d\r\nCOLOR = %d  RGB:%d,%d,%d = JUDGE:%d\r\nRESULT = %d\r\n-----------------\r\n", num, obj, red, green, blue, judgement, location[num]);
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
    if (check_type == 1){
        if (distance < 7) {
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

    fprintf(bt, "h:%f s:%f v:%f max:%f min%f\r\n----------------\r\n",h, s, v, max, min);
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
    ev3_sensor_config(PortSensorColor1, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor2, HT_NXT_COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor3, HT_NXT_COLOR_SENSOR);
    ev3_sensor_config(PortUltrasonic, ULTRASONIC_SENSOR);




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
    float distance_test = 0;
    uint8_t obj2;
    uint8_t obj1;
    rgb_raw_t rgb_val1;
    rgb_raw_t rgb_val2;
    float red1;
    float blue1;
    float green1;
    float judge1;
    float red2;
    float blue2;
    float green2;
    float judge2;
    
    while (true) {
        tslp_tsk(1000*MSEC);
        while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_3, &test)) {
            ;
        }  
        while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_3, &obj1)) {
            ;
        }  
        while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_3, &testrgb)) {
            ;
        }  
        while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_3, &rgb_val1)) {
            ;
        } 
        while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_2, &test)) {
            ;
        }  
        while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_2, &obj2)) {
            ;
        }  
        while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_2, &testrgb)) {
            ;
        }  
        while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_2, &rgb_val2)) {
            ;
        }  
        distance_test = ev3_ultrasonic_sensor_get_distance(EV3_PORT_4);
        red1 = rgb_val1.r;
        green1 = rgb_val1.g;
        blue1 = rgb_val1.b;

        red2 = rgb_val2.r;
        green2 = rgb_val2.g;
        blue2 = rgb_val2.b;

        judge1 = red1 + green1 + blue1;
        judge2 = red2 + green2 + blue2;
        fprintf(bt, "----------------\r\nSENSOR3: color=%d R%f G%f B%f JUD%f\r\n", obj1, red1, green1, blue1, judge1);
        fprintf(bt, "SENSOR2: color=%d R%f G%f B%f JUD%f\r\n", obj2, red2, green2, blue2, judge2);
        fprintf(bt, "SONIC:%f\r\n", distance_test);
        hsv(0,0);
        
    }
    

    int A = 0;
    int B = 0;
    int C = 0;
    int D = 0;

    //ev3_motor_reset_counts(EV3_PORT_A);
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    tslp_tsk(1000*MSEC);
    straight(100, 40);
    tslp_tsk(1000*MSEC);
    straight(-100, 40);
    tslp_tsk(1000*MSEC);
    straight(100, 40);
    tslp_tsk(1000*MSEC);
    straight(-100, 40);
    tslp_tsk(1000*MSEC);
    straight(100, 40);
    tslp_tsk(1000*MSEC);
    straight(-100, 40);
    //ev3_motor_reset_counts(EV3_PORT_D);
    //ev3_motor_rotate(EV3_PORT_A, 3600, -100, false);
    //ev3_motor_rotate(EV3_PORT_B, 2800, -100, false);
    //ev3_motor_rotate(EV3_PORT_C, 2800, -100, false);
    tslp_tsk(10000*MSEC);
    newsteering(-20, 45);
    newsteering(20, 45);
    tslp_tsk(8000*MSEC);
    newsteering(20, 90);
    tslp_tsk(8000*MSEC);
    newsteering(25, 90);
    tslp_tsk(8000*MSEC);
    newsteering(30, 90);
    //ev3_motor_rotate(EV3_PORT_D, 3600, -100, false);
    tslp_tsk(20000*MSEC);
    //ev3_motor_stop(EV3_PORT_A,true);
    ev3_motor_stop(EV3_PORT_B,true);
    ev3_motor_stop(EV3_PORT_C,true);
    //ev3_motor_stop(EV3_PORT_D,true);
    //A = ev3_motor_get_counts(EV3_PORT_A);
    B = ev3_motor_get_counts(EV3_PORT_B);
    C = ev3_motor_get_counts(EV3_PORT_C);
    //D = ev3_motor_get_counts(EV3_PORT_D);
    //ev3_motor_stop(EV3_PORT_A,true);
    ev3_motor_stop(EV3_PORT_B,true);
    ev3_motor_stop(EV3_PORT_C,true);
    //ev3_motor_stop(EV3_PORT_D,true);
    fprintf(bt, "A:%d\r\nB:%d\r\nC:%d\r\nD:%d\r\n------------\r\n", A, B, C, D);
    while (true) {
    }
    
    
   

   
   
    start = 2;
    

    switch (start){
        case 2:
            newsteering(-60, 80);
            steering_time(1600, -25, 0);
            tank_turn(180, 0, 30);
            tank_turn(180, 30, -30);
            tank_turn(60, -30, 0);
            tank_turn(58, 0, -30);
            steering(-90, 60, 4);


            break;
        default:
            newsteering(-90, 80);
            break;
    }
            
    tslp_tsk(100*MSEC);
    newsteering(-50, 11);
    tslp_tsk(300*MSEC);
    p_turn(90, 0, 1);
    steering_time(800, -40, 0);
    p_turn(92, -1, 1);
    tslp_tsk(200*MSEC);
    steering_color(COLOR_WHITE, 35, 0);
    steering_color(COLOR_BLACK, 35, 0);
    steering_time(700, 15, 0);
    linetrace_length(24, 9);
    /*steering_time(900, -25, 0);
    tslp_tsk(200*MSEC);
    newsteering(20, 4.3);
    tslp_tsk(500*MSEC);
    p_turn(88, 0, 1);
    newsteering(25, 9);
    tslp_tsk(500*MSEC);*/
    newsteering(20, 4.1);
    tslp_tsk(500*MSEC);
    
    ev3_motor_reset_counts(EV3_PORT_D);
    
    map_check(8, RIGHT);
    chemical_taker(8, RIGHT);
    tslp_tsk(600*MSEC);
    newsteering(70, 36.7);
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
        tslp_tsk(1000*MSEC);
        p_turn(88, 1, -1);
        chemical_took(11, RIGHT);
        steering_time(800, 25, 0);        
    }
    
    newsteering(-50, 13.8);
    tslp_tsk(400*MSEC);
    p_turn(90, 1, -1);
    tslp_tsk(400*MSEC);
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
        tslp_tsk(400*MSEC);
        p_turn(59, 1, 0);
        tslp_tsk(200*MSEC);
    }
    else{
        p_turn(60, 0, 1);
        tslp_tsk(400*MSEC);
        p_turn(59, 1, 0);
        tslp_tsk(200*MSEC);
        newsteering(35, 10);
    }
    map_check(4, LEFT);
    chemical_taker(4, LEFT);
    water(4);
    water(7);
    tslp_tsk(200*MSEC);
    newsteering(70, 39.2);
    map_check(3, LEFT);
    chemical_taker(3, LEFT);
    steering_time(1000, 30, 0);
    tslp_tsk(200*MSEC);
    tank_turn(76, 0, -30);
    tslp_tsk(200*MSEC);
    p_turn(52, 1, -1);
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
        if(chemical < 1)ev3_motor_rotate(EV3_PORT_A, 280, 20, false);
        steering_time(1200, -30, 0);
        newsteering(70, 49.7);
        tslp_tsk(500*MSEC);
        p_turn(90, -1, 1);
        tslp_tsk(200*MSEC);
        newsteering(-40, 32);
        map_check(6, RIGHT);
        chemical_taker(6, RIGHT);
        newsteering(60, 37);
        map_check(5, RIGHT);
        chemical_taker(5, RIGHT);
        newsteering(70, 65);
        tslp_tsk(200*MSEC);
        p_turn(90, 1, -1);
        tslp_tsk(200*MSEC);
        newsteering(-30, 8);
    }    
    else if (white == FIRE){
        location[5] = FIRE;
        newsteering(-30, 6);
        p_turn(90, -1, 1);
        steering_time(700, 30, 0);
        newsteering(-70, 38);
        tslp_tsk(400*MSEC);
        p_turn(90, 1, -1);
        water(5);
        tslp_tsk(700*MSEC);
        newsteering(80, 72);
        tslp_tsk(200*MSEC);
        p_turn(90, 1, -1);
        newsteering(-30, 8);
    }
    else if (white == PERSON || white == NOTHING) {
        newsteering(80, 68);
        tslp_tsk(200*MSEC);
        p_turn(90, 1, -1);
        steering_time(1500, -30, 0);
        newsteering(70, 40);
    }
    else {
        newsteering(-35, 6);
        tslp_tsk(500*MSEC);
        p_turn(90, 1, -1);
        if(chemical < 1)ev3_motor_rotate(EV3_PORT_A, 280, 20, false);
        steering_time(1200, -30, 0);
        newsteering(70, 49.7);
        tslp_tsk(500*MSEC);
        p_turn(89, -1, 1);
        tslp_tsk(200*MSEC);
        newsteering(30, 5);
        map_check(5, RIGHT);
        chemical_taker(5, RIGHT);
        newsteering(-40, 37);
        map_check(6, RIGHT);
        chemical_taker(6, RIGHT);
        newsteering(70, 102);
        tslp_tsk(200*MSEC);
        p_turn(90, 1, -1);
        tslp_tsk(200*MSEC);
        newsteering(-30, 8);
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
    if(chemical_type == RIGHT)newsteering(-70, 62);

    steering_time(700, -30, 0);
    tslp_tsk(200*MSEC);

    steering_time(200, 30, 0);
    tank_turn(80, 30, 0);
    tslp_tsk(200*MSEC);
    tank_turn(98, 0, -30);
    steering(-65, 80, 0);
    steering_time(2000, -30, 0);
    



    


    newsteering(50, 17);
    tank_turn(140, 0, 45);
    tslp_tsk(200*MSEC);
    tank_turn(160, 45, -45);
    steering_time(1000, 25, 0);
    

    newsteering(-70, 65);
    ev3_motor_rotate(EV3_PORT_D, 90, 30, true);
    tslp_tsk(200*MSEC);
    if (location[0] == PERSON || location[1] == PERSON) map[0] = 1; /*blue*/
    if (location[2] == PERSON || location[3] == PERSON) map[1] = 1; /*green*/
    if (location[5] == PERSON || location[6] == PERSON) map[2] = 1; /*white*/
    if (location[4] == PERSON || location[7] == PERSON) map[3] = 1; /*yellow*/
    if (location[8] == PERSON || location[9] == PERSON) map[4] = 1; /*brown*/
    if (location[10] == PERSON || location[11] == PERSON) map[5] = 1; /*red*/

    switch (start) {
    case 1:
        tank_turn(185, -30, 0);
        tslp_tsk(400*MSEC);
        tank_turn(184, 0, -30);
        tslp_tsk(200*MSEC);
        newsteering(70, 23.8);
        
        
    
        
        if (map[4] == 1 || map[5] == 1) {
            if (map[4] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, 40, true); /*時計回りがプラス*/
                newsteering(30, 1.7);
                ev3_motor_rotate(EV3_PORT_D, 40, -10, true);
                ev3_motor_rotate(EV3_PORT_D, 10, 40, true);
                newsteering(30, 4.8);
            }
            else newsteering(50, 6.5);
            if (map[5] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, -40, true); 
                newsteering(30, 2.6);
                ev3_motor_rotate(EV3_PORT_D, 40, 25, true);
                ev3_motor_rotate(EV3_PORT_D, 10, -40, true);
                newsteering(30, 5.4);
            }
            else newsteering(50, 8);
        }
        else newsteering(50, 14.5);

        if (map[2] == 1 || map[3] == 1) {
            if (map[2] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, 40, true); 
                newsteering(30, 1.7);
                ev3_motor_rotate(EV3_PORT_D, 40, -10, true);
                ev3_motor_rotate(EV3_PORT_D, 10, 40, true);
                newsteering(30, 4.8);
            }
            else newsteering(50, 6.5);
            if (map[3] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, -40, true); 
                newsteering(30, 2.6);
                ev3_motor_rotate(EV3_PORT_D, 40, 25, true);
                ev3_motor_rotate(EV3_PORT_D, 10, -40, true);
                newsteering(30, 5.4);
            }
            else newsteering(50, 8);
        }
        else newsteering(50, 14.5);
        
        if (map[0] == 1 || map[1] == 1) {
            if (map[0] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, 40, true); 
                newsteering(30, 1.7);
                ev3_motor_rotate(EV3_PORT_D, 40, -10, true);
                ev3_motor_rotate(EV3_PORT_D, 10, 40, true);
                newsteering(30, 4.8);
            }
            else newsteering(50, 6.5);
            if (map[1] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, -40, true); 
                newsteering(30, 2.6);
                ev3_motor_rotate(EV3_PORT_D, 40, 25, true);
                ev3_motor_rotate(EV3_PORT_D, 10, -40, true);
            }
        }
        steering_time(3000, 30, -40);
        break;
    
    
    
    
    
    
    
    case 2:
        newsteering(70, 36);
        tslp_tsk(300*MSEC);
        p_turn(180, 1, 0);
        tslp_tsk(300*MSEC);
    
    
        if (map[0] == 1 || map[1] == 1) {
            if (map[1] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, 40, true); /*時計回りがプラス*/
                newsteering(30, 2.2);
                ev3_motor_rotate(EV3_PORT_D, 40, -10, true);
                ev3_motor_rotate(EV3_PORT_D, 10, 40, true);
                newsteering(30, 4.3);
            }
            else newsteering(50, 6.5);
            if (map[0] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, -40, true);
                newsteering(30, 2.6);
                ev3_motor_rotate(EV3_PORT_D, 40, 10, true);
                ev3_motor_rotate(EV3_PORT_D, 10, -40, true);
                newsteering(30, 5.4);
            }
            else newsteering(50, 8);
        }
        else newsteering(50, 14);
    
        if (map[2] == 1 || map[3] == 1) {
            if (map[3] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, 40, true);
                newsteering(30, 2.2);
                ev3_motor_rotate(EV3_PORT_D, 40, -10, true);
                ev3_motor_rotate(EV3_PORT_D, 10, 40, true);
                newsteering(30, 4.3);
            }
            else newsteering(50, 6);
            if (map[2] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, -40, true);
                newsteering(30, 2.6);
                ev3_motor_rotate(EV3_PORT_D, 40, 10, true);
                ev3_motor_rotate(EV3_PORT_D, 10, -40, true);
                newsteering(30, 5.4);
            }
            else newsteering(50, 8);
        }
        else newsteering(50, 14);
    
        if (map[4] == 1 || map[5] == 1) {
            if (map[5] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, 40, true);
                newsteering(30, 2.2);
                ev3_motor_rotate(EV3_PORT_D, 40, -10, true);
                ev3_motor_rotate(EV3_PORT_D, 10, 40, true);
                newsteering(30, 4.3);
            }
            else newsteering(50, 7);
            if (map[4] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, -40, true);
                newsteering(30, 2.6);
                ev3_motor_rotate(EV3_PORT_D, 40, 10, true);
                ev3_motor_rotate(EV3_PORT_D, 10, -40, true);
            }
        }
    
        steering_time(3000, 30, 30);

        break;
    }
    
    
    

    


    

}