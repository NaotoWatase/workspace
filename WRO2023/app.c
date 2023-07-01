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
int num_marking;
int num_4step;
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
    int steer = 0;
    int lb_power;
    int rc_power;
    int now_right_angle = 0;
    int now_left_angle = 0;
    int now_angle = 0; 
    int diff = 0;
    int maximum = abs(power);
    float changing_power = 13;
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
        if (changing_power < maximum && goal_angle - (6*ROBOT1CM) > now_angle) changing_power = changing_power + 0.003;
        if (goal_angle - (6*ROBOT1CM) <= now_angle) changing_power = changing_power - 0.01;
        if (changing_power <= 13) changing_power = 13;
        if (goal_angle <= now_angle) break;   
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}
void turn(int angle, int lb_power, int rc_power){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int lb_sign = lb_power / abs(lb_power);
    int rc_sign = rc_power / abs(rc_power);
    int now_right_angle = 0;
    int now_left_angle = 0;
    int maximum = 80;
    int points = 35;
    float turn_num = 0.1531;
    if (abs(lb_power) == 30 && abs(rc_power) == 30) {
        turn_num = 0.1531;
        points = 35;
    }

    if (lb_power < 0 && rc_power == 0) turn_num = 0.151;
    if (lb_power > 0 && rc_power == 0) turn_num = 0.1536;
    if (lb_power == 0 && rc_power < 0) turn_num = 0.152;
    if (abs(lb_power) >= abs(rc_power)) maximum = abs(lb_power);
    if (abs(rc_power) > abs(lb_power)) maximum = abs(rc_power);
    float changing_power = 15;
    int goal_angle = angle*turn_num*ROBOT1CM;
    while (true) {
        now_left_angle = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_right_angle = abs(ev3_motor_get_counts(EV3_PORT_C));
        
        if (changing_power <= 15) changing_power = 15;
        if (lb_power == 0) {
            if (changing_power < maximum && goal_angle - (points*turn_num*ROBOT1CM) > now_right_angle) changing_power = changing_power + 0.007;
            if (goal_angle - (points*turn_num*ROBOT1CM) <= now_right_angle) changing_power = changing_power - 0.17;
            if (changing_power <= 20) changing_power = 20;
            if (goal_angle <= now_right_angle) break; 
            rc_power = changing_power*rc_sign;
            ev3_motor_set_power(EV3_PORT_C, rc_power);
        }
        if (rc_power == 0) {
            if (changing_power < maximum && goal_angle - (points*turn_num*ROBOT1CM) > now_left_angle) changing_power = changing_power + 0.007;
            if (goal_angle - (points*turn_num*ROBOT1CM) <= now_left_angle) changing_power = changing_power - 0.17;
            if (changing_power <= 20) changing_power = 20;
            if (goal_angle <= now_left_angle) break; 
            lb_power = -changing_power*lb_sign;
            ev3_motor_set_power(EV3_PORT_B, lb_power);

        }
        if (lb_power != 0 && rc_power != 0){
            if (changing_power < maximum && goal_angle - (points*turn_num*ROBOT1CM) > now_right_angle) changing_power = changing_power + 0.004;
            if (goal_angle - (points*turn_num*ROBOT1CM) <= now_right_angle) changing_power = changing_power - 0.15;
            if (changing_power <= 15) changing_power = 15;
            if (goal_angle <= now_right_angle) break; 
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
    if(lb_power < 0) {
        while (true) {
            color_2 = ev3_color_sensor_get_color(EV3_PORT_2);
            if(color_2 == COLOR_WHITE)break;
        }
        while (true) {
            color_2 = ev3_color_sensor_get_color(EV3_PORT_2);
            if(color_2 == COLOR_BLACK)break;
        }
    }
    if(lb_power > 0) {
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
            if(green - blue > 9) location[num] = 3;
            else location[num] = 2;
            break;
        default:
            break;
    } 
    if(num <= 1)location_t[num] = location[num];
    else location_f[num - 2] = location[num];
    fprintf(bt, "RESULT = T=%d,%d F=%d,%d,%d,%d\r\n-----------------\r\n", location_t[0], location_t[1], location_f[0], location_f[1], location_f[2], location_f[3]);
}

void check_pattern(){
    num_marking = location_t[0] * 10 + location_t[1];
    if(num_marking == 32) num_marking = 23;
    num_4step = location_f[0] * 1000 + location_f[1] * 100 + location_f[2] * 10 + location_f[3];
    switch (num_marking){
    case 33:
        switch (num_4step){
        case 3223:
            pattern = 3;
            break;
        case 3232:
            pattern = 5;
            break;
        case 3322:
            pattern = 2;
            break;
        case 2323:
            pattern = 6;
            break;
        case 2233:
            pattern = 1;
            break;
        case 2332:
            pattern = 4;
            break;
        }
        break;
    case 22:
        switch (num_4step){
        case 3223:
            pattern = 4;\
            break;
        case 3232:
            pattern = 6;
            break;
        case 3322:
            pattern = 1;
            break;
        case 2323:
            pattern = 5;
            break;
        case 2233:
            pattern = 2;
            break;
        case 2332:
            pattern = 3;
        break;
        }
        break;
    case 23:
        switch (num_4step){
        case 3223:
        case 3232:
        case 2323:
        case 2332:
            pattern = 1;
            break;
        case 3322:
        case 2233:
            pattern = 3;
            break;
        }
        break;
    }
}
    
void stopping(){
    while(ev3_button_is_pressed(ENTER_BUTTON) == false) {}    
    tslp_tsk(2000*MSEC);
}

void arm_take_obj(){
    int now_counts = 0;
    int goal_counts = 270 - arm_set;
    arm_set = 0;
    ev3_motor_reset_counts(EV3_PORT_A);
    ev3_motor_set_power(EV3_PORT_A, 25);
    while (true) {
        now_counts = ev3_motor_get_counts(EV3_PORT_A);
        if(now_counts > (goal_counts - 20))break;
    }
    ev3_motor_set_power(EV3_PORT_A, 40);
    tslp_tsk(300*MSEC);
    ev3_motor_stop(EV3_PORT_A, true);
}

void arm_take_ship(){
    int now_counts = 0;
    int goal_counts = 90;
    ev3_motor_reset_counts(EV3_PORT_A);
    ev3_motor_set_power(EV3_PORT_A, 40);
    while (true) {
        now_counts = ev3_motor_get_counts(EV3_PORT_A);
        if(now_counts > (goal_counts - 2))break;
    }
    ev3_motor_set_power(EV3_PORT_A, 15);
    tslp_tsk(390*MSEC);
    ev3_motor_set_power(EV3_PORT_A, 10);
}

void arm_mode_change(armmode_t mode) {
    switch (mode) {
        case SET:
            if(now_arm_angle >= 0)ev3_motor_set_power(EV3_PORT_D, 20);
            else ev3_motor_set_power(EV3_PORT_D, -20);
            break;
        case LEFTDOWN:
            ev3_motor_set_power(EV3_PORT_D, 7);
            break;
        case RIGHTDOWN:
            ev3_motor_set_power(EV3_PORT_D, -7);
            break;
        default:
            break;
    }
    while (true) {
        now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
        if(now_arm_angle >= 30 && mode == LEFTDOWN) break;
        if(now_arm_angle <= -30 && mode == RIGHTDOWN) break;
        if((now_arm_angle >= -2 || now_arm_angle <= 2) && mode == SET) break;
    }
    if(mode == LEFTDOWN) {
        ev3_motor_set_power(EV3_PORT_D, 30);
        tslp_tsk(200*MSEC);
    }
    if(mode == RIGHTDOWN) {
        ev3_motor_set_power(EV3_PORT_D, -30);
        tslp_tsk(200*MSEC);
    }
    ev3_motor_stop(EV3_PORT_D, true);
    now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
}

void arm_reset_A(){
    ev3_motor_set_power(EV3_PORT_A, -50);
    tslp_tsk(200*MSEC);
    ev3_motor_set_power(EV3_PORT_A, -30);
    tslp_tsk(500*MSEC);
    ev3_motor_stop(EV3_PORT_A, true);
    tslp_tsk(100*MSEC);
    ev3_motor_rotate(EV3_PORT_A, 5, 10, true);
    ev3_motor_stop(EV3_PORT_A, true);
}

void arm_set_A(int arm_angle){
    ev3_motor_rotate(EV3_PORT_A, arm_angle, 30, true);
    arm_set = arm_angle + arm_set;
}

void obj_nkc(){
    switch (pattern){
    case 1: 
        //pattern1 version1
        straight(1.5, -30);
        turn(90, -30, 30);
        arm_set_A(70);
        straight(7, 30);
        //objとる
        arm_set_A(35);
        straight(7, -30);
        arm_take_obj();
        arm_reset_A();
        arm_set_A(70);
        turn(90, 30, -30);
        straight(14, 30);
        turn(90, -30, 30);
        straight(7, 30);
        //objとる
        arm_set_A(35);
        straight(7, -30);
        arm_take_obj();
        arm_reset_A();
        turn(90, -30, 30);
        straight(7, 30);
        turn(90, -30, 30);
        break;
    case 2:
        //pattern1 version2
        straight(12.5, 30);
        turn(90, -30, 30);
        arm_set_A(70);
        straight(7, 30);
        //objとる
        arm_set_A(35);
        straight(7, -30);
        arm_take_obj();
        arm_reset_A();
        arm_set_A(70);
        turn(90, -30, 30);
        straight(14.5, 30);
        turn(90, 30, -30);
        straight(7, 30);
        //objとる
        arm_set_A(35);
        straight(7, -30);
        arm_take_obj();
        arm_reset_A();
        turn(90, 30, -30);
        straight(7, 30);
        turn(90, 30, -30);
        break;
    case 3: 
        //pattern2 version1
        straight(5.5, 30);
        turn(90, -30, 30);
        arm_set_A(70);
        straight(7, 30);
        //objとる
        arm_set_A(35);
        straight(7, -30);
        arm_take_obj();
        arm_reset_A();
        arm_set_A(70);
        turn(100, 0, -30);
        turn(100, -30, 0);
        straight(20.5, 30);
        //objとる
        arm_set_A(35);
        straight(7, -30);
        arm_take_obj();
        arm_reset_A();
        arm_set_A(70);
        turn(90, 30, -30);
        straight(14, 30);
        turn(90, -30, 30);
        straight(7, 30);
        //objとる
        arm_set_A(35);
        straight(7, -30);
        arm_take_obj();
        arm_reset_A();
        turn(90, -30, 30);
        straight(7, 30);
        turn(90, -30, 30);
        break;
    case 4:
        //pattern2 version2
        straight(9, -30);
        turn(90, -30, 30);
        arm_set_A(70);
        straight(7, 30);
        //objとる
        arm_set_A(35);
        straight(7, -30);
        arm_take_obj();
        arm_reset_A();
        arm_set_A(70);
        turn(90, 30, -30);
        straight(28.5, 30);
        turn(90, -30, 30);
        straight(7, 30);
        //objとる
        arm_set_A(35);
        straight(7, -30);
        arm_take_obj();
        arm_reset_A();
        arm_set_A(70);
        turn(90, -30, 30);
        straight(14, 30);
        turn(90, 30, -30);
        straight(7, 30);
        //objとる
        arm_set_A(35);
        straight(7, -30);
        arm_take_obj();
        arm_reset_A();
        turn(180, -30, 30);
        break;
    case 5: 
        //pattern3
        straight(20, 30);
        turn(90, -30, 30);
        arm_set_A(70);
        straight(7, 30);
        //objとる
        arm_set_A(35);
        straight(7, -30);
        arm_take_obj();
        arm_reset_A();
        arm_set_A(70);
        turn(90, -30, 30);
        straight(21.5, 30);
        turn(90, 30, -30);
        straight(7, 30);
        //objとる
        arm_set_A(35);
        straight(7, -30);
        arm_take_obj();
        arm_reset_A();
        arm_set_A(70);
        turn(100, -30, 0);
        turn(100, 0, -30);
        straight(20.5, 30);
        //objとる
        arm_set_A(35);
        straight(7, -30);
        arm_take_obj();
        arm_reset_A();
        turn(180, 30, -30);
        break;
    case 6:
        //pattern4
        straight(9, -30);
        turn(90, -30, 30);
        arm_set_A(70);
        straight(7, 30);
        //objとる
        arm_set_A(35);
        arm_take_obj();
        straight(7, -30);
        arm_reset_A();
        arm_set_A(70);
        turn(90, 30, -30);
        straight(21, 30);
        turn(90, -30, 30);
        straight(7, 30);
        //objとる
        arm_set_A(35);
        arm_take_obj();
        straight(7, -30);
        arm_reset_A();
        arm_set_A(70);
        turn(100, 0, -30);
        turn(100, -30, 0);
        straight(20.5, 30);
        //objとる
        arm_set_A(35);
        arm_take_obj();
        straight(7, -30);
        arm_reset_A();
        turn(180, 30, -30);
        break;
    }
}

void linetrace_cm_pd(int power, float gain, float d_gain, float cm){
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
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void gain_set(int power, float *p_gain, float *d_gain){
    *p_gain = 0.3;
    *d_gain = 10;
    if(power == 20){
        *p_gain = 0.6;   
        *d_gain = 30;   
    }
    if(power == 30){
        *p_gain = 0.25;   
        *d_gain = 30;   
    }
    if(power == 40){
        *p_gain = 0.25;   
        *d_gain = 30;   
    }
    if(power == 50){
        *p_gain = 0.2;   
        *d_gain = 25;   
    }
    if(power == 60){
        *p_gain = 0.15;   
        *d_gain = 15;   
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

void linetrace_cm_pd_SP(float cm, int power){
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
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void linetrace_color_pd_SP(port_t port, colorid_t color, int power){
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

    /* ここからコーディング */
    battery = ev3_battery_voltage_mV();
    fprintf(bt, "BATTERY:%d", battery);

    stopping();


    straight(6, 30);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 20);
    straight(13, 30);
    //船押して燃料補給

    turn(70, 0, -30);
    turn(70, -30, 0);
    straight(6, -30);
    //ここで色を読む
    obj_check(1, LEFT);
    straight(5, -30);
    //ここで色を読む
    obj_check(0, LEFT);
    turn(180, 30, 0);
    arm_reset_A();
    linetrace_cm_pd_SP(38, 30);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 20);
    straight(6.5, 30);
    turn(90, -30, 30);
    //ev3_motor_rotate();
    linetrace_cm_pd_SP(10, 20);
    linetrace_cm_pd_SP(16.5, 20);
    turn(180, 30, 0);
    straight(4.5, -30);
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

    //patternの確認
    check_pattern();

    turn(85, 30, 0);
    turn(85, 0, 30);

    //pattenごとの動き
    obj_nkc();

    linetrace_cm_pd_SP(10, 30);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 20);
    linetrace_cm_pd_SP(20, 30);
    linetrace_cm_pd_SP(5.5, 20);
    //armで船掴む
    arm_take_ship();

    straight(19, -30);
    turn(90, -30, 30);
    linetrace_cm_pd_SP(12, 20);
    linetrace_cm_pd_SP(33, 20);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 20);
    //arm開く
    arm_reset_A();

    turn(180, 30, -30);
    straight(19.5, -15);
    straight(3, 15);
    //armおろしてオブジェクト下ろす
    arm_mode_change(LEFTDOWN);
    tslp_tsk(200*MSEC);
    arm_mode_change(RIGHTDOWN);
    tslp_tsk(200*MSEC);
    arm_mode_change(LEFTDOWN);
    tslp_tsk(200*MSEC);
    turn(180, 30, 0);
    linetrace_cm_pd_SP(21, 20);
    linetrace_cm_pd_SP(8, 20);
    arm_take_obj();
    //armでオブジェクトとる
    straight(37, -30);
    turn(90, -30, 30);
    arm_reset_A();
    linetrace_cm_pd_SP(10, 20);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 25);
    linetrace_cm_pd_SP(35, 30);
    turn(90, 30, -30);
    straight(32, 30);
    turn(90, -30, 30);
    straight(7.5, 15);
    arm_take_ship();
    //armで船掴む
    turn(90, -20, 20);
    straight(32.5, 30);
    turn(90, -20, 20);
    linetrace_cm_pd_SP(15, 20);
    linetrace_cm_pd(30, 0.1, 25, 125);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30);
    turn(90, 0, -25);
    straight(18.5, 40);
    turn(45, 15, -15);
    linetrace_cm_pd_SP(12, 20);
    //arm戻す
    arm_reset_A();
    turn(180, 30, -30);
    straight(3, -20);
    arm_mode_change(RIGHTDOWN);
    tslp_tsk(200*MSEC);
    arm_mode_change(LEFTDOWN);
    turn(100, 30, 0);
    turn(100, 0, 30);
    straight(14, -30);
    arm_mode_change(RIGHTDOWN);

    turn(100, 0, 30);
    turn(100, 30, 0);
    linetrace_cm_pd_SP(45, 30);
    straight(9, 50);
    arm_take_obj();


    battery = ev3_battery_voltage_mV();
    fprintf(bt, "BATTERY:%d", battery);


}
