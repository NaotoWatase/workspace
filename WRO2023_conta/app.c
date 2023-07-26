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
#include "math.h"

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

static FILE *bt = NULL;

FILE *file;
const char* logfilename = "/wro_result.txt";

/**
 * Define the connection ports of the sensors and motors.
 * By default, this application uses the following ports:
 * Touch sensor: Port 2
 * Color sensor: Port 3
 * Left motor:   Port B
 * Right motor:  Port C
 */
static const sensor_port_t  PortSensorColor1 = EV3_PORT_1;
static const sensor_port_t  PortSensorColor2 = EV3_PORT_2;
static const sensor_port_t  PortSensorColor3 = EV3_PORT_3;
static const sensor_port_t  PortSensorColor4 = EV3_PORT_4;

static const motor_port_t   PortMotorArmUp   = EV3_PORT_A;
static const motor_port_t   PortMotorLeft    = EV3_PORT_B;
static const motor_port_t   PortMotorRight   = EV3_PORT_C;
static const motor_port_t   PortMotorArmDown = EV3_PORT_D;
colorid_t obj = 0;
rgb_raw_t testrgb;
rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)

int location_t[2] = {0, 0};
int location_f[4] = {0, 0, 0, 0};
int location[6] = {0, 0, 0, 0, 0, 0};
int location_aim[4] = {2, 2, 2, 2};
int pattern;

int arm_set = 0;
float h = 0;
float s = 0;
float v = 0;
float min = 0;
float max = 0;
float red = 0;
float green = 0;
float blue = 0;
float judgement = 0;


armmode_t now_armmode = SET;
int now_arm_angle = 0;

int battery;


void straight(float cm, int power);
void turn(int angle, int lb_power, int rc_power);
void steering_color(colorid_t color_stop, int power, int steering);
void start();
void obj_check(int num, port_t sensor);
void obj_measure(int num, port_t sensor);
void obj_know(int num);
void gain_set(int power, float *p_gain, float *d_gain);

void pattern1122();
void pattern2211();
void pattern1221();
void pattern2112();
void pattern1212();
void pattern2121();




void stopping();

static void button_clicked_handler(intptr_t button) {
    switch(button) {
    case BACK_BUTTON:
#if !defined(BUILD_MODULE)
        syslog(LOG_NOTICE, "Back button clicked.");
#endif
        break;
    }
}

void straight(float cm, int power){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    //if (power == 30) power = 40;
    int steer = 0;
    int lb_power;
    int rc_power;
    int now_right_angle = 0;
    int now_left_angle = 0;
    int now_angle = 0; 
    int diff = 0;
    int maximum = abs(power);
    float changing_power = 14;
    int sign = power/abs(power);
    int goal_angle = cm * ROBOT1CM;
    while (true){
        now_left_angle = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_right_angle = abs(ev3_motor_get_counts(EV3_PORT_C));
        now_angle = (now_right_angle + now_left_angle) / 2;
        diff = now_right_angle - now_left_angle;
        steer = (diff*4);
        if(steer > 0) {
            lb_power = changing_power;
            rc_power = changing_power - (changing_power * steer / 50);
            rc_power = sign*rc_power;
            lb_power = -sign*lb_power;
        }
        else {
            lb_power = changing_power + (changing_power * steer / 50);
            rc_power = changing_power;
            rc_power = sign*rc_power;
            lb_power = -sign*lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (changing_power < maximum && goal_angle - (6*ROBOT1CM) > now_angle) changing_power = changing_power + 0.005;
        if (goal_angle - (6*ROBOT1CM) <= now_angle) changing_power = changing_power - 0.01;
        if (changing_power <= 14) changing_power = 14;
        if (goal_angle <= now_angle) break;   
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void straight_color(port_t sensor, colorid_t color, int power){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    //if (power == 30) power = 40;
    int steer = 0;
    int lb_power;
    int rc_power;
    int now_right_angle = 0;
    int now_left_angle = 0;
    int diff = 0;
    int maximum = abs(power);
    float changing_power = 14;
    int sign = power/abs(power);
    colorid_t l2_color;
    colorid_t r3_color;
    while (true){
        l2_color = ev3_color_sensor_get_color(EV3_PORT_2);
        r3_color = ev3_color_sensor_get_color(EV3_PORT_3);
        now_left_angle = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_right_angle = abs(ev3_motor_get_counts(EV3_PORT_C));
        diff = now_right_angle - now_left_angle;
        steer = (diff*4);
        if(steer > 0) {
            lb_power = changing_power;
            rc_power = changing_power - (changing_power * steer / 50);
            rc_power = sign*rc_power;
            lb_power = -sign*lb_power;
        }
        else {
            lb_power = changing_power + (changing_power * steer / 50);
            rc_power = changing_power;
            rc_power = sign*rc_power;
            lb_power = -sign*lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, -lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (changing_power < maximum) changing_power = changing_power + 0.003;
        if (changing_power <= 14) changing_power = 14;
        if (color == l2_color && color == r3_color && sensor == BOTH) break;   
        if (color == r3_color && sensor == RIGHT) break;   
        if (color == l2_color && sensor == LEFT) break;   
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void straight_on(int power){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    ev3_motor_set_power(EV3_PORT_B, -power);
    ev3_motor_set_power(EV3_PORT_C, power);
}

void straight_off(float cm, bool_t logic){
    int now_motor_angle_rc = 0;
    while (true) {
        now_motor_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_B));
        if(cm*ROBOT1CM <= now_motor_angle_rc) break;
    }
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    if (logic == true) {
        ev3_motor_stop(EV3_PORT_B, true);
        ev3_motor_stop(EV3_PORT_C, true);
    }
}

void turn(int angle, int lb_power, int rc_power){
    if (lb_power == 30) lb_power = lb_power + 35;
    if (lb_power == -30) lb_power = lb_power - 35;
    if (rc_power == 30) rc_power = rc_power + 35;
    if (rc_power == -30) rc_power = rc_power - 35;

    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int lb_sign = lb_power / abs(lb_power);
    int rc_sign = rc_power / abs(rc_power);
    int now_right_angle = 0;
    int now_left_angle = 0;
    int average = 0;
    int maximum = 80;
    int points = 30;
    float turn_num = 0.153;
    if (abs(lb_power) != 0 && abs(rc_power) != 0) {
        turn_num = 0.152;
    }
    if (abs(lb_power) >= abs(rc_power)) maximum = abs(lb_power);
    if (abs(rc_power) > abs(lb_power)) maximum = abs(rc_power);
    float changing_power = 15;
    int goal_angle = angle*turn_num*ROBOT1CM;
    while (true) {
        now_left_angle = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_right_angle = abs(ev3_motor_get_counts(EV3_PORT_C));
        average = (now_left_angle + now_right_angle) / 2;
        
        if (changing_power <= 15) changing_power = 15;
        if (lb_power == 0) {
            if (changing_power < maximum && goal_angle - (points*turn_num*ROBOT1CM) > now_right_angle) changing_power = changing_power + 0.006;
            if (goal_angle - ((points + 5)*turn_num*ROBOT1CM) <= now_right_angle) changing_power = changing_power - 0.17;
            if (changing_power <= 10) changing_power = 10;
            if (goal_angle <= now_right_angle) break; 
            rc_power = changing_power*rc_sign;
            ev3_motor_set_power(EV3_PORT_C, rc_power);
        }
        if (rc_power == 0) {
            if (changing_power < maximum && goal_angle - (points*turn_num*ROBOT1CM) > now_left_angle) changing_power = changing_power + 0.006;
            if (goal_angle - ((points + 5)*turn_num*ROBOT1CM) <= now_left_angle) changing_power = changing_power - 0.17;
            if (changing_power <= 10) changing_power = 10;
            if (goal_angle <= now_left_angle) break; 
            lb_power = -changing_power*lb_sign;
            ev3_motor_set_power(EV3_PORT_B, lb_power);
        }
        if (lb_power != 0 && rc_power != 0){
            if (changing_power < maximum && goal_angle - (points*turn_num*ROBOT1CM) > average) changing_power = changing_power + 0.004;
            if (goal_angle - (points*turn_num*ROBOT1CM) <= now_right_angle) changing_power = changing_power - 0.15;
            if (changing_power <= 10) changing_power = 10;
            if (goal_angle <= average) break; 
            rc_power = changing_power*rc_sign;
            lb_power = -changing_power*lb_sign;
            ev3_motor_set_power(EV3_PORT_C, rc_power);
            ev3_motor_set_power(EV3_PORT_B, lb_power);
        }  
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void turn_color(int lb_power, int rc_power){
    colorid_t color_2;
    colorid_t color_3;
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    lb_power = -lb_power;
    ev3_motor_set_power(EV3_PORT_C, rc_power);
    ev3_motor_set_power(EV3_PORT_B, lb_power);
    if(lb_power < 0 || rc_power < 0) {
        while (true) {
            color_2 = ev3_color_sensor_get_color(EV3_PORT_2);
            if(color_2 == COLOR_WHITE)break;
        }
        while (true) {
            color_2 = ev3_color_sensor_get_color(EV3_PORT_2);
            if(color_2 == COLOR_BLACK)break;
        }
    }
    else if(lb_power > 0 || rc_power > 0) {
        while (true) {
            color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
            if(color_3 == COLOR_WHITE)break;
        }
        while (true) {
            color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
            if(color_3 == COLOR_BLACK)break;
        }
    }
    
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void linetrace_cm(int power, float gain, float cm){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int now_angle_lb = 0;
    int now_angle_rc = 0;
    int average = 0;
    int lb_power;
    int rc_power;
    int now_reflect_2; 
    int now_reflect_3; 
    int last_diff = 0;
    int diff = 0;
    float d;
    int steering;
    while (true) {
        
        now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        now_angle_lb = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_C));
        average = (now_angle_lb + now_angle_rc) / 2;
        last_diff = diff;
        diff = now_reflect_2 - now_reflect_3;
        d = (diff - last_diff) / 0.005 * 0.027;
        steering = diff * gain + d;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);

        if (average >= ROBOT1CM*cm) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);

}

void linetrace_one_side_cm(int power, float gain, float cm, port_t oneside){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int now_angle_lb = 0;
    int now_angle_rc = 0;
    int average = 0;
    int lb_power;
    int rc_power;
    int now_reflect_2; 
    int now_reflect_3; 
    int last_diff = 0;
    int diff = 0;
    float d;
    int steering;
    while (true) {
        now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        now_angle_lb = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_C));
        average = (now_angle_lb + now_angle_rc) / 2;
        switch (oneside) {
        case LEFT:
            diff = now_reflect_2 - 20;
            gain = -gain;
            break;
        case RIGHT:
            diff = now_reflect_3 - 20;
            break;
        default:
            diff = now_reflect_2 - now_reflect_3;
            break;
        }
        last_diff = diff;
        d = (diff - last_diff) / 0.005;
        steering = diff * gain + d * 0;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);

        if (average >= ROBOT1CM*cm) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void linetrace_one_side_color(int power, float gain, port_t oneside, colorid_t color){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int lb_power;
    int rc_power;
    int now_reflect_2; 
    int now_reflect_3;
    colorid_t color_2 = 0;
    colorid_t color_3 = 0; 

    int last_diff = 0;
    int diff = 0;
    float d;
    int steering;
    while (true) {
        now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        if(color != COLOR_BLACK) {
            color_2 = ev3_color_sensor_get_color(EV3_PORT_2);
            color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        }
        switch (oneside) {
        case LEFT:
            diff = now_reflect_2 - 20;
            gain = -gain;
            break;
        case RIGHT:
            diff = now_reflect_3 - 20;
            break;
        default:
            diff = now_reflect_2 - now_reflect_3;
            break;
        }
        last_diff = diff;
        d = (diff - last_diff) / 0.005;
        steering = diff * gain + d * 0;

        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (oneside == RIGHT && color == COLOR_BLACK && now_reflect_3 <= 8)break;
        if (oneside == LEFT && color == COLOR_BLACK && now_reflect_2 <= 8)break;
        if (oneside == BOTH && color == COLOR_BLACK && now_reflect_3 <= 10 && now_reflect_2 <= 10)break;
        if (oneside == RIGHT && color_3 == color && color != COLOR_BLACK)break;
        if (oneside == LEFT && color_2 == color && color != COLOR_BLACK)break;
        if (oneside == BOTH && color_2 == color && color_3 == color && color != COLOR_BLACK)break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void linetrace_color(int power, float gain, port_t port, colorid_t color){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int lb_power;
    int rc_power;
    int now_reflect_2; 
    int now_reflect_3;
    colorid_t color_2 = 0;
    colorid_t color_3 = 0; 

    int diff;
    int steering;
    while (true) {
        now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        if(color != COLOR_BLACK) {
            color_2 = ev3_color_sensor_get_color(EV3_PORT_2);
            color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        }
        diff = now_reflect_2 - now_reflect_3;
        steering = diff * gain;

        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (port == RIGHT && color == COLOR_BLACK && now_reflect_3 <= 8)break;
        if (port == LEFT && color == COLOR_BLACK && now_reflect_2 <= 8)break;
        if (port == BOTH && color == COLOR_BLACK && now_reflect_3 <= 10 && now_reflect_2 <= 10)break;
        if (port == RIGHT && color_3 == color && color != COLOR_BLACK)break;
        if (port == LEFT && color_2 == color && color != COLOR_BLACK)break;
        if (port == BOTH && color_2 == color && color_3 == color && color != COLOR_BLACK)break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}


void steering_reflect(int reflect_stop, int power, int steering){
    int reflect_2;
    int reflect_3;
    if(steering > 0) {
        (void)ev3_motor_set_power(EV3_PORT_B, -power);
        (void)ev3_motor_set_power(EV3_PORT_C, power+(power*steering/50));
    }
    else {
        (void)ev3_motor_set_power(EV3_PORT_B, -(power-(power*steering/50)));
        (void)ev3_motor_set_power(EV3_PORT_C, power);
    }
    while (true) {  
        reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        if(reflect_stop >= reflect_2 && reflect_stop >= reflect_3) break;
    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);    
}

void test_turn(int lb_power, int rc_power, int angle){
    ev3_motor_rotate(EV3_PORT_B, 90*0.1532*ROBOT1CM, 30, false);
    ev3_motor_rotate(EV3_PORT_C, 90*0.1532*ROBOT1CM, 30, true);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void obj_check(int num, port_t sensor){
    obj_measure(num, sensor);
    obj_know(num);

    //tslp_tsk(300*MSEC);
}

void obj_measure(int num, port_t sensor) {
    if(sensor == LEFT){
        obj = ev3_color_sensor_get_color(EV3_PORT_1);
        ev3_color_sensor_get_rgb_raw(EV3_PORT_1, &rgb_val);
    } 
    else{ 
        obj = ev3_color_sensor_get_color(EV3_PORT_4);
        ev3_color_sensor_get_rgb_raw(EV3_PORT_4, &rgb_val);
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

    fprintf(bt, "LOCATION = %d\r\nCOLOR = %d\r\nRGB:%f,%f,%f = JUDGE:%f\r\nHSV:%f,%f,%f = MAX:%f MIN:%f\r\n", num, obj, red, green, blue, judgement, h, s, v, max, min);
    file=fopen(logfilename,"a");//ファイルをオープン(名前の指定)
    fprintf(file, "LOCATION = %d\r\nCOLOR = %d\r\nRGB:%f,%f,%f = JUDGE:%f\r\nHSV:%f,%f,%f = MAX:%f MIN:%f\r\n", num, obj, red, green, blue, judgement, h, s, v, max, min);
    fclose(file);
}

void obj_know(int num){
    switch(obj){
        case 2:
            location[num] = 2;
            break;
        case 3:
            location[num] = 3;
            break;
        case 0:
        case 1:
        case 4:
        case 5:
        case 6:
            if(h < 150) location[num] = 3;
            else location[num] = 2;
            break;
        default:
            break;
    } 
    if(num <= 1)location_t[num] = location[num];
    else location_f[num - 2] = location[num];
    fprintf(bt, "RESULT = T=%d,%d F=%d,%d,%d,%d\r\n-----------------\r\n", location_t[0], location_t[1], location_f[0], location_f[1], location_f[2], location_f[3]);
    file=fopen(logfilename,"a");//ファイルをオープン(名前の指定)
    fprintf(file, "RESULT = T=%d,%d F=%d,%d,%d,%d\r\n-----------------\r\n", location_t[0], location_t[1], location_f[0], location_f[1], location_f[2], location_f[3]);
    fclose(file);
}

void check_pattern(){
    int target_1;
    int target_2;
    int target_1_count = 1;
    int target_2_count = 1;
    target_1 = 5 - location_t[0];
    target_2 = 5 - location_t[1];
    if (location_f[0] == target_1 && target_1_count == 1) {
        target_1_count = target_1_count - 1;
        location_aim[0] = 1;
    }
    else if (location_f[0] == target_2 && target_2_count == 1) {
        target_2_count = target_2_count - 1;
        location_aim[0] = 1;
    }
    if (location_f[1] == target_1 && target_1_count == 1) {
        target_1_count = target_1_count - 1;
        location_aim[1] = 1;
    }
    else if (location_f[1] == target_2 && target_2_count == 1) {
        target_2_count = target_2_count - 1;
        location_aim[1] = 1;
    }
    if (location_f[2] == target_1 && target_1_count == 1) {
        target_1_count = target_1_count - 1;
        location_aim[2] = 1;
    }
    else if (location_f[2] == target_2 && target_2_count == 1) {
        target_2_count = target_2_count - 1;
        location_aim[2] = 1;
    }
    if (location_f[3] == target_1 && target_1_count == 1) {
        target_1_count = target_1_count - 1;
        location_aim[3] = 1;
    }
    else if (location_f[3] == target_2 && target_2_count == 1) {
        target_2_count = target_2_count - 1;
        location_aim[3] = 1;
    }
    pattern = location_aim[0] * 1000 + location_aim[1] * 100 + location_aim[2] * 10 + location_aim[3];
}
    
void stopping(){
    while(ev3_button_is_pressed(ENTER_BUTTON) == false) {}    
    tslp_tsk(2000*MSEC);
}

void arm_take_obj(){
    /*int now_counts = 0;
    int goal_counts = 270 - arm_set;
    arm_set = 0;
    ev3_motor_reset_counts(EV3_PORT_A);
    ev3_motor_set_power(EV3_PORT_A, 15);
    while (true) {
        now_counts = ev3_motor_get_counts(EV3_PORT_A);
        if(now_counts > (goal_counts - 20))break;
    }
    ev3_motor_set_power(EV3_PORT_A, 18);
    tslp_tsk(600*MSEC);
    ev3_motor_stop(EV3_PORT_A, true);
    */
    ev3_motor_set_power(EV3_PORT_A, 45);
}

void arm_take_ship(){
    //int now_counts = 0;
    //int goal_counts = 90;
    ev3_motor_reset_counts(EV3_PORT_A);
    /*ev3_motor_set_power(EV3_PORT_A, 60);
    while (true) {
        now_counts = ev3_motor_get_counts(EV3_PORT_A);
        if(now_counts > (goal_counts - 20))break;
    }*/
    ev3_motor_set_power(EV3_PORT_A, 40);
    tslp_tsk(500*MSEC);
    ev3_motor_set_power(EV3_PORT_A, 70);
}

void arm_mode_change(armmode_t mode) {
    switch (mode) {
        case SET:
            if(now_arm_angle >= 0)ev3_motor_set_power(EV3_PORT_D, -20);
            else ev3_motor_set_power(EV3_PORT_D, 20);
            break;
        case LEFTDOWN:
            ev3_motor_set_power(EV3_PORT_D, 8);
            break;
        case RIGHTDOWN:
            ev3_motor_set_power(EV3_PORT_D, -8);
            break;
        default:
            break;
    }
    while (true) {
        now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
        if(now_arm_angle >= 30 && mode == LEFTDOWN) break;
        if(now_arm_angle <= -30 && mode == RIGHTDOWN) break;
        if(now_arm_angle >= -2 && now_arm_angle <= 2 && mode == SET) break;
    }
    if(mode == LEFTDOWN) {
        ev3_motor_set_power(EV3_PORT_D, 7);
        tslp_tsk(200*MSEC);
        ev3_motor_stop(EV3_PORT_D, true);
        tslp_tsk(100*MSEC);
        ev3_motor_set_power(EV3_PORT_D, 30);
        tslp_tsk(200*MSEC);
    }
    if(mode == RIGHTDOWN) {
        ev3_motor_set_power(EV3_PORT_D, -7);
        tslp_tsk(200*MSEC);
        ev3_motor_stop(EV3_PORT_D, true);
        tslp_tsk(100*MSEC);
        ev3_motor_set_power(EV3_PORT_D, -30);
        tslp_tsk(200*MSEC);
    }
    ev3_motor_stop(EV3_PORT_D, true);
    now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
}

void arm_reset_A(){
    /*
    ev3_motor_set_power(EV3_PORT_A, -20);
    tslp_tsk(350*MSEC);
    ev3_motor_set_power(EV3_PORT_A, -40);
    tslp_tsk(600*MSEC);
    ev3_motor_stop(EV3_PORT_A, true);
    tslp_tsk(100*MSEC);
    ev3_motor_rotate(EV3_PORT_A, 5, 10, true);
    ev3_motor_stop(EV3_PORT_A, true);
    */
    //ev3_motor_set_power(EV3_PORT_A, -30);
    ev3_motor_rotate(EV3_PORT_A, 400, -30, false);
}

void arm_set_A(int arm_angle, bool_t stop){
    ev3_motor_stop(EV3_PORT_A, true);
    int power = 30;
    if (arm_angle == 30) power = 40;
    ev3_motor_rotate(EV3_PORT_A, arm_angle, power, stop);
    arm_set = arm_angle + arm_set;
}

void linetrace_cm_pd(int power, float gain, float d_gain, float cm, bool_t stop){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int now_angle_lb = 0;
    int now_angle_rc = 0;
    int average = 0;
    int lb_power;
    int rc_power;
    int now_reflect_2; 
    int now_reflect_3; 
    int last_diff = 0;
    int diff = 0;
    float d;
    int steering;

    now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
    now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
    diff = now_reflect_2 - now_reflect_3;
    while (true) {
        
        now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        now_angle_lb = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_C));
        average = (now_angle_lb + now_angle_rc) / 2;
        last_diff = diff;
        diff = now_reflect_2 - now_reflect_3;
        d = (diff - last_diff);
        steering = diff * gain + d * d_gain;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);

        if (average >= ROBOT1CM*cm) break;
    }
    ev3_motor_stop(EV3_PORT_B, stop);
    ev3_motor_stop(EV3_PORT_C, stop);
}

void gain_set(int power, float *p_gain, float *d_gain){
    *p_gain = 0.3;
    *d_gain = 10;
    if(power == 10){
        *p_gain = 0.7;   
        *d_gain = 55;   
    }
    if(power == 20){
        *p_gain = 0.75;   
        *d_gain = 55;   
    }
    if(power == 24){
        *p_gain = -0.5;   
        *d_gain = 55;   
    }
    if(power == 30){
        *p_gain = 0.4;   
        *d_gain = 80;   
    }
    if(power == 40){
        *p_gain = 0.25;   
        *d_gain = 70;   
    }
    if(power == 50){
        *p_gain = 0.15;   
        *d_gain = 60;   
    }
    if(power == 60){
        *p_gain = 0.16;   
        *d_gain = 70;   
    }
    if(power == 70){
        *p_gain = 0.15;   
        *d_gain = 15;   
    }
    if(power == 80){
        *p_gain = 0.15;
        *d_gain = 10;
    }
}

void linetrace_cm_pd_SP(float cm, int power, bool_t stop){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int now_angle_lb = 0;
    int now_angle_rc = 0;
    int average = 0;
    int lb_power;
    int rc_power;
    int now_reflect_2; 
    int now_reflect_3; 
    int last_diff = 0;
    int diff = 0;
    float d;
    int steering;
    float p_gain;
    float d_gain;
    gain_set(power, &p_gain, &d_gain);

    now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
    now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
    diff = now_reflect_2 - now_reflect_3;
    while (true) {
        
        now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        now_angle_lb = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_C));
        average = (now_angle_lb + now_angle_rc) / 2;
        last_diff = diff;
        diff = now_reflect_2 - now_reflect_3;
        d = (diff - last_diff);
        steering = diff * p_gain + d * d_gain;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);

        if (average >= ROBOT1CM*cm) break;
    }
    ev3_motor_stop(EV3_PORT_B, stop);
    ev3_motor_stop(EV3_PORT_C, stop);
}

void linetrace_color_pd_SP(port_t port, colorid_t color, int power, bool_t stop){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int lb_power;
    int rc_power;
    int now_reflect_2; 
    int now_reflect_3; 
    colorid_t color_2 = 0;
    colorid_t color_3 = 0; 
    int last_diff = 0;
    int diff = 0;
    float d;
    int steering;
    float p_gain;
    float d_gain;
    gain_set(power, &p_gain, &d_gain);

    now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
    now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
    diff = now_reflect_2 - now_reflect_3;
    if (port == RIGHT && power == 24) {
        diff = ev3_color_sensor_get_reflect(EV3_PORT_C) - 25;
    }
    if (port == LEFT && power == 24) {
        diff = ev3_color_sensor_get_reflect(EV3_PORT_C) - 25;
    }
    while (true) {
        
        now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        if(color != COLOR_BLACK) {
            color_2 = ev3_color_sensor_get_color(EV3_PORT_2);
            color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        }
        last_diff = diff;
        diff = now_reflect_2 - now_reflect_3;
        d = (diff - last_diff);
        steering = diff * p_gain + d * d_gain;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (port == RIGHT && color == COLOR_BLACK && now_reflect_3 <= 8 && power != 24)break;
        if (port == LEFT && color == COLOR_BLACK && now_reflect_2 <= 8 && power != 24)break;
        if (port == BOTH && color == COLOR_BLACK && now_reflect_3 <= 13 && now_reflect_2 <= 13)break;
        if (port == RIGHT && color_3 == color && color != COLOR_BLACK)break;
        if (port == LEFT && color_2 == color && color != COLOR_BLACK)break;
        if (port == BOTH && color_2 == color && color_3 == color && color != COLOR_BLACK)break;
        if (port == RIGHT && color == COLOR_BLACK && now_reflect_3 <= 13 && now_reflect_2 <= 13 && power == 24)break;
        if (port == LEFT && color == COLOR_BLACK && now_reflect_3 <= 13 && now_reflect_2 <= 13 && power == 24)break;
        
    }
    ev3_motor_stop(EV3_PORT_B, stop);
    ev3_motor_stop(EV3_PORT_C, stop);
}

void start_nkc(){
    straight(6, 50);
    linetrace_color_pd_SP(RIGHT, COLOR_BLACK, 24, false);
    straight(12.5, 30);
    //船押して燃料補給

    /*
    turn(70, 0, -70);
    turn(70, -70, 0);
    */
    
    /*
    straight(4, -40);
    //ここで色を読む
    obj_check(1, LEFT);
    straight(5, -40);
    //ここで色を読む
    obj_check(0, LEFT);
    */
    
    
    straight_on(-30);
    straight_off(1, false);
    obj_check(1, LEFT);
    straight_off(5, false);
    obj_check(0, LEFT);
    straight_off(4, true);

    turn(90, 80, -80);
}

void objprepare_nkc(){
    linetrace_cm_pd_SP(8, 30, false);
    linetrace_cm_pd_SP(30, 60, false);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 40, false);
    linetrace_cm_pd_SP(6.5, 30, true);
    turn(90, -30, 30);
    //ev3_motor_rotate();
    linetrace_cm_pd_SP(4, 20, false);
    linetrace_cm_pd_SP(15, 30, false);
    linetrace_cm_pd_SP(8.5, 10, true);
    tslp_tsk(200*MSEC);
    turn(180, 55, 0);
    tslp_tsk(100*MSEC);

    straight(0.7, 30);
    obj_check(4, LEFT);
    straight_on(-20);
    straight_off(7.2, false);
    obj_check(3, LEFT);
    straight_off(7.4, false);
    obj_check(2, LEFT);
    straight_off(0.7, true);
    tslp_tsk(200*MSEC);
    location[5] = 10 - location[4] - location[3] - location[2];
    location_f[3] = location[5];
    
    /*
    straight(3.6, -30);
    //ここで色を読む
    obj_check(5, LEFT);
    straight(7, -30);
    //ここで色を読む
    obj_check(4, LEFT);
    straight(7, -30);
    //ここで色を読む
    obj_check(3, LEFT);
    straight(7, -30);
    //ここで色を読む
    obj_check(2, LEFT);
    */
}

void obj_nkc(){
    //patternの確認
    check_pattern();
    /*turn(83, 26, 0);
    turn(83, 0, 26);*/

    switch (pattern){
    case 1122: 
        pattern1122();
        break;
    case 2211:
        pattern2211();
        break;
    case 2112: 
        pattern2112();
        break;
    case 1221:
        pattern1221();
        break;
    case 2121: 
        pattern2121();
        break;
    case 1212:
        pattern1212();
        break;
    default:
        pattern1122();
        break;
    }
}

void pattern1122(){
    //straight(1, -20);
    turn(90, -30, 30);
    straight(8, -30);
    arm_reset_A();
    tslp_tsk(800*MSEC);
    ev3_motor_stop(EV3_PORT_A, true);
    arm_set_A(95, true);
    straight(11.5, 20);
    //objとる
    arm_set_A(30, true);
    arm_take_obj();
    straight(11, -20);
    turn(90, 30, -30);
    arm_reset_A();
    straight(14.9, 30);
    turn(90, -30, 30);
    ev3_motor_stop(EV3_PORT_A, true);
    arm_set_A(95, true);
    straight(11.5, 20);
    //objとる
    arm_set_A(30, true);
    arm_take_obj();
    straight(11.5, -20);
    turn(90, -30, 30);
    straight(7.5, 30);
    turn(90, -30, 30);
    arm_reset_A();
}

void pattern2211(){
   straight(14, 30);
   turn(90, -30, 30);
   straight(8, -30);
   arm_reset_A();
   tslp_tsk(800*MSEC);
   arm_set_A(95, true);
   straight(11.5, 20);
   //objとる
   arm_set_A(30, true);
   arm_take_obj();
   straight(11, -20);
   turn(90, -30, 30);
   arm_reset_A();
   straight(15, 30);
   turn(90, 30, -30);
   arm_set_A(95, true);
   straight(11.5, 20);
   //objとる
   arm_set_A(30, true);
   arm_take_obj();
   straight(11.5, -20);
   turn(90, 30, -30);
   straight(8, 30);
   turn(90, 30, -30);
   arm_reset_A();
}

void pattern2112(){
   straight(7.1, 30);
   turn(90, -30, 30);
   straight(8, -30);
   arm_reset_A();
   tslp_tsk(800*MSEC);
   arm_set_A(95, true);
   straight(11.5, 20);
   //objとる
   arm_set_A(30, true);
   arm_take_obj();
   straight(11, -20);
   arm_reset_A();
   turn(90, -30, 30);
   straight(8, 20);
   turn(90, 30, -30);
   arm_set_A(95, true);
   straight(11.5, 20);
   //objとる
   arm_set_A(30, true);
   arm_take_obj();
   straight(11, -20);
   turn(90, 30, -30);
   arm_reset_A();
   straight(14.9, 30);
   turn(90, -30, 30);
   arm_set_A(95, true);
   straight(11.5, 20);
   //objとる
   arm_set_A(30, true);
   arm_take_obj();
   straight(11.5, -20);
   turn(90, -30, 30);
   straight(8, 30);
   turn(90, -30, 30);
   arm_reset_A();
}

void pattern1221(){
   straight(8.1, -30);
   turn(90, -30, 30);
   straight(8, -20);
   arm_reset_A();
   tslp_tsk(800*MSEC);
   arm_set_A(95, true);
   straight(11.5, 20);
   //objとる
   arm_set_A(30, true);
   arm_take_obj();
   straight(11, -20);
   turn(90, 30, -30);
   arm_reset_A();
   straight(29.7, 30);
   turn(90, -30, 30);
   arm_set_A(95, true);
   straight(11.5, 20);
   //objとる
   arm_set_A(30, true);
   arm_take_obj();
   straight(11, -20);
   turn(90, -30, 30);
   arm_reset_A();
   straight(15, 30);
   turn(90, 30, -30);
   arm_set_A(95, true);
   straight(11.5, 20);
   //objとる
   arm_set_A(30, true);
   arm_take_obj();
   straight(11.5, -20);
   turn(180, -30, 30);
   arm_reset_A();
}


void pattern2121(){
   straight(20.5, 30);
   turn(90, -30, 30);
   straight(7, -20);
   arm_reset_A();
   tslp_tsk(800*MSEC);
   arm_set_A(95, true);
   straight(11.5, 20);
   //objとる
   arm_set_A(30, true);
   arm_take_obj();
   straight(11, -20);
   turn(90, -30, 30);
   arm_reset_A();
   straight(21.9, 30);
   turn(90, 30, -30);
   arm_set_A(95, true);
   straight(11.5, 20);
   //objとる
   arm_set_A(30, true);
   arm_take_obj();
   straight(11, -20);
   arm_reset_A();
   turn(90, 30, -30);
   straight(7.7, 20);
   turn(90, -30, 30);
   arm_set_A(95, true);
   straight(11.5, 20);
   //objとる
   arm_set_A(30, true);
   arm_take_obj();
   straight(11.5, -20);
   turn(180, 30, -30);
   arm_reset_A();
}


void pattern1212(){
   straight(8.1, -30);
   turn(90, -30, 30);
   straight(7, -20);
   arm_reset_A();
   tslp_tsk(800*MSEC);
   arm_set_A(95, true);
   straight(11.5, 20);
   //objとる
   arm_set_A(30, true);
   arm_take_obj();
   straight(11, -20);
   turn(90, 30, -30);
   arm_reset_A();
   straight(22.4, 30);
   turn(90, -30, 30);
   arm_set_A(95, true);
   straight(11.5, 20);
   //objとる
   arm_set_A(30, true);
   arm_take_obj();
   straight(11.5, -20);
   arm_reset_A();
   turn(90, -30, 30);
   straight(7.6, 20);
   turn(90, 30, -30);
   arm_set_A(95, true);
   straight(11.5, 20);
   //objとる
   arm_set_A(30, true);
   arm_take_obj();
   straight(11.5, -20);
   turn(180, 30, -30);
   arm_reset_A();
}


void smallship_nkc(){
    linetrace_cm_pd_SP(10, 20, false);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 40, false);
    arm_set_A(115, false);
    linetrace_cm_pd_SP(16, 50, false);
    linetrace_cm_pd_SP(8, 10, true);
    /*linetrace_cm_pd_SP(20, 40, false);
    linetrace_cm_pd_SP(5.5, 20, true);*/
    //armで船掴む
    tslp_tsk(200*MSEC);
    arm_take_ship();

    straight(17, -40);
    turn(90, -30, 30);
    linetrace_cm_pd(15, 0.6, 55, 28, false);
    //straight(15, 15);
    //linetrace_cm_pd(50, 0.15, 50, 15, false);
    //linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30, false);
    /*linetrace_cm_pd(20, 0.6, 55, 15, false);
    linetrace_cm_pd(50, 0.15, 50, 25, false);
    linetrace_cm_pd(20, 0.6, 55, 10, true);*/
    straight(66, 80);
    tslp_tsk(100*MSEC);
    turn(194, 0, 50);
    //arm開く

    arm_reset_A();
    tslp_tsk(300*MSEC);
    turn(187, 30, -30);
    straight(6, -15);
    //armおろしてオブジェクト下ろす
    arm_mode_change(RIGHTDOWN);
    tslp_tsk(100*MSEC);
    arm_mode_change(LEFTDOWN);
    tslp_tsk(100*MSEC);
    arm_mode_change(RIGHTDOWN);
    tslp_tsk(100*MSEC);
    straight_on(25);
    while (true)
    {
        if(ev3_color_sensor_get_color(EV3_PORT_2) == COLOR_WHITE && ev3_color_sensor_get_color(EV3_PORT_3) == COLOR_WHITE) break;
    }
    
    straight(9.5, 30);
    turn(90, 30, -30);
}


void whiteobj_nkc(){
    linetrace_cm_pd_SP(20, 30, false);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 60, false);
    linetrace_cm_pd_SP(6.2, 60, true);
    turn(90, 30, -30);
    arm_set_A(85, false);
    linetrace_cm_pd_SP(19, 30, false);
    linetrace_cm_pd_SP(16.5, 60, true);
    tslp_tsk(100*MSEC);
    arm_set_A(40, true);
    tslp_tsk(100*MSEC);
    arm_take_obj();
    //armでオブジェクトとる
    straight(35.5, -60);
    turn(90, -30, 30);
}

void bigprepare_nkc(){
    arm_reset_A();
    linetrace_cm_pd_SP(15, 20, false);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 60, false);
    arm_set_A(100, false);
    linetrace_cm_pd_SP(35, 30, true);
    turn(90, 30, -30);
    straight(32.5, 50);
    turn(90, -30, 30);
    straight(7.5, 15);
    /* SPEED UP
    arm_reset_A();
    turn(90, -30, 30);
    straight(103, 80);
    */
}

void bigship_nkc(){
    arm_take_ship();
    tslp_tsk(100*MSEC);
    //armで船掴む
    straight(75, -50);
    tslp_tsk(100*MSEC);
    turn(190, 0, 40);
    straight(21.5, 30);
    turn_color(-20, 20);
    linetrace_cm_pd(20, 0.7, 70, 32, false);
    straight(60, 80);
    //linetrace_cm_pd(40, 0.2, 45, 80, false);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30, false);
    straight(6.5, 30);
    turn(90, 24, -24);
    linetrace_cm_pd_SP(8.5, 20, false);
    linetrace_cm_pd_SP(13, 30, true);
    tslp_tsk(100*MSEC);
    straight(13, -30);
    //arm戻す
    arm_reset_A();
    tslp_tsk(300*MSEC);
    turn(195, 30, -30);
    straight(7, -15);
    arm_mode_change(LEFTDOWN);
    turn(15, -30, 30);
    arm_mode_change(RIGHTDOWN);
    ev3_motor_set_power(EV3_PORT_A, 40);
    tslp_tsk(400*MSEC);
    arm_mode_change(LEFTDOWN);
    tslp_tsk(200*MSEC);
    /*arm_mode_change(SET);
    tslp_tsk(200*MSEC);*/


    /*turn(180, 30, -30);
    straight(5, -15);
    arm_mode_change(LEFTDOWN);
    tslp_tsk(200*MSEC);
    turn(90, 70, 0);
    turn(90, 0, 70);
    straight(13.5, -30);
    arm_mode_change(RIGHTDOWN);
    ev3_motor_set_power(EV3_PORT_A, 40);
    tslp_tsk(400*MSEC);
    arm_mode_change(LEFTDOWN);
    tslp_tsk(100*MSEC);
    arm_mode_change(SET);
    tslp_tsk(200*MSEC);*/
}

void goal_nkc(){
    straight(3, 30);
    linetrace_cm_pd_SP(14.5, 30, false);
    straight(30, 70);
    //完全ゴール　straight(35, 70);

}

void main_task(intptr_t unused) {

    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Register button handlers */
    ev3_button_set_on_clicked(BACK_BUTTON, &button_clicked_handler, BACK_BUTTON);

    /* Configure motors */
    ev3_motor_config(PortMotorLeft, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorRight, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorArmDown, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorArmUp, MEDIUM_MOTOR);

    /* Configure sensors */
    ev3_sensor_config(PortSensorColor1, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor2, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor3, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor4, COLOR_SENSOR);

    fprintf(bt, "----GAME_START----\r\n");
    file=fopen(logfilename,"a");//ファイルをオープン(名前の指定)
    fprintf(file, "----GAME_START----\r\n");
    fclose(file);
    

    /* ここからコーディング */

    //turnは回転する時軸から回転した側に0.5cmずれる

    battery = ev3_battery_voltage_mV();
    fprintf(bt, "BATTERY:%d\r\n", battery);
    file=fopen(logfilename,"a");//ファイルをオープン(名前の指定)
    fprintf(file, "BATTERY:%d\r\n", battery);
    fclose(file);

    while(ev3_button_is_pressed(ENTER_BUTTON) == false) {}    
    tslp_tsk(500*MSEC);

    /*while (true)
    {
        arm_mode_change(RIGHTDOWN);
        tslp_tsk(100*MSEC);
        arm_mode_change(LEFTDOWN);
        tslp_tsk(1500*MSEC);
    }*/

    start_nkc();
    //マーキングの色読んでターンまでまで　アームは上
    objprepare_nkc();
    //オブジェクトの色読むまで　アームは下
    obj_nkc();
    //オブジェクトを取る　アームは下
    smallship_nkc();
    //ライントレースからsmallshipとって運びオブジェクトを置く　アームは下
    whiteobj_nkc();
    //白のオブジェクトとって回転まで　アームは上
    bigprepare_nkc();
    //bigship取るまで　アームは下
    bigship_nkc();
    //bigship置いてオブジェクト置くまで　アームは上
    goal_nkc();
   

    battery = ev3_battery_voltage_mV();
    fprintf(bt, "BATTERY:%d\r\n", battery);
    file=fopen(logfilename,"a");//ファイルをオープン(名前の指定)
    fprintf(file, "BATTERY:%d\r\n", battery);
    fclose(file);


}
