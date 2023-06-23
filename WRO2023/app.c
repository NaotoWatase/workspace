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

float h = 0;
float s = 0;
float v = 0;
float min = 0;
float max = 0;
float red = 0;
float green = 0;
float blue = 0;
float judgement = 0;

//fprintf(bt, "LOCATION = %d\r\nCOLOR = %d\r\nRGB:%f,%f,%f = JUDGE:%f\r\nHSV:%f,%f,%f = MAX:%f MIN:%f\r\nDISTANCE:%f\r\nRESULT = %d\r\n-----------------\r\n", num, obj, red, green, blue, judgement, h, s, v, max, min, obj_distance, location[num]);

armmode_t now_armmode = SET;
int now_arm_angle = 0;

int battery;


void straight(float cm, int power);
void turn(int angle, int lb_power, int rc_power);
void steering_color(colorid_t color_stop, int power, int steering);
void start();
void obj_check(int num, port_t sensor);
void obj_measure(port_t sensor);
void obj_know(int num);




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
    int points = 30;
    float turn_num = 0.1531;
    if (lb_power < 0 && rc_power == 0) turn_num = 0.151;
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
            if (goal_angle - (points*turn_num*ROBOT1CM) <= now_right_angle) changing_power = changing_power - 0.1;
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
    obj_measure(sensor);
    obj_know(num);

    //tslp_tsk(300*MSEC);
}

void obj_measure(port_t sensor) {
    if(sensor == LEFT){
        obj = ev3_color_sensor_get_color(EV3_PORT_2);
        ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
    } 
    else{ 
        obj = ev3_color_sensor_get_color(EV3_PORT_3);
        ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
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
    switch(obj){
        case 0:
        case 1:
        case 2:
            location[num] = 2;
        case 3:
            location[num] = 3;
        case 4:
        case 5:
        case 6:
        default:
            break;
    } 
    if(num <= 1)location_t[num] = location[num];
    else location_f[num - 2] = location[num];
}

void stopping(){
    while(ev3_button_is_pressed(ENTER_BUTTON) == false) {}    
    tslp_tsk(2000*MSEC);
}

void arm_take_obj(){
    int now_counts = 0;
    int goal_counts = 270;
    ev3_motor_reset_counts(EV3_PORT_A);
    ev3_motor_set_power(EV3_PORT_A, 30);
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
    ev3_motor_set_power(EV3_PORT_A, 10);
    tslp_tsk(300*MSEC);
}

void arm_mode_change(armmode_t mode) {
    switch (mode) {
        case SET:
            if(now_arm_angle >= 0)ev3_motor_set_power(EV3_PORT_D, -20);
            else ev3_motor_set_power(EV3_PORT_D, 20);
            break;
        case RIGHTDOWN:
            ev3_motor_set_power(EV3_PORT_D, 20);
            break;
        case LEFTDOWN:
            ev3_motor_set_power(EV3_PORT_D, -20);
            break;
    default:
        break;
    }
    while (true) {
        now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
        if(now_arm_angle >= 60 && mode == RIGHTDOWN) break;
        if(now_arm_angle <= -60 && mode == LEFTDOWN) break;
        if((now_arm_angle >= -4 || now_arm_angle <= 4) && mode == SET) break;
    }
    ev3_motor_stop(EV3_PORT_D, true);
    now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
}

void arm_reset_A(){
    ev3_motor_set_power(EV3_PORT_A, -30);
    tslp_tsk(900*MSEC);
    ev3_motor_stop(EV3_PORT_A, true);
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
    stopping();



    straight(6, 30);
    linetrace_color(20, 0.5, BOTH, COLOR_BLACK);
    straight(10.5, 30);
    //船押して燃料補給

    straight(13.5, -30);
    //ここで色を読む
    straight(5, -30);
    //ここで色を読む

    turn(180, 30, 0);
    arm_reset_A();
    linetrace_cm(30, 0.35, 38);
    linetrace_color(25, 0.4, BOTH, COLOR_BLACK);
    straight(6, 30);
    turn(90, -30, 30);
    //ev3_motor_rotate();
    linetrace_cm(25, 0.4, 16);
    linetrace_cm(15, 0.6, 10.5);
    turn(180, 30, 0);
    straight(6, -30);
    //ここで色を読む
    straight(7, -30);
    //ここで色を読む
    straight(7, -30);
    //ここで色を読む
    straight(7, -30);
    //ここで色を読む
    //何やかんやあってオブジェクトとりました


    turn(85, 30, 0);
    turn(85, 0, 30);
    stopping();

    //pattern1 version1
    turn(90, -30, 30);
    straight(7, 30);
    //armの角度を調整した方がいい
    arm_take_obj();
    straight(7, -30);
    arm_reset_A();
    turn(90, 30, -30);
    straight(14, 30);
    turn(90, -30, 30);
    straight(7, 30);
    //armの角度を調整した方がいい
    arm_take_obj();
    straight(7, -30);
    arm_reset_A();
    turn(90, -30, 30);
    straight(7, 30);
    turn(90, -30, 30);

    stopping();

    //pattern1 version2
    straight(14, 30);
    turn(90, -30, 30);
    straight(7, 30);
    //armの角度を調整した方がいい
    arm_take_obj();
    straight(7, -30);
    arm_reset_A();
    turn(90, -30, 30);
    straight(14.5, 30);
    turn(90, 30, -30);
    straight(7, 30);
    //armの角度を調整した方がいい
    arm_take_obj();
    straight(7, -30);
    arm_reset_A();
    turn(90, 30, -30);
    straight(7, 30);
    turn(90, 30, -30);

    
    stopping();

    //pattern2 version1
    straight(7, 30);
    turn(90, -30, 30);
    straight(7, 30);
    //armの角度を調整した方がいい
    arm_take_obj();
    straight(7, -30);
    arm_reset_A();
    turn(100, 0, -30);
    turn(100, -30, 0);
    straight(22, 30);
    //armの角度を調整した方がいい
    arm_take_obj();
    straight(7, -30);
    arm_reset_A();
    turn(90, 30, -30);
    straight(14, 30);
    turn(90, -30, 30);
    straight(7, 30);
    //armの角度を調整した方がいい
    arm_take_obj();
    straight(7, -30);
    arm_reset_A();
    turn(90, -30, 30);
    straight(7, 30);
    turn(90, -30, 30);


    stopping();

    //pattern2 version2
    straight(7.5, -30);
    turn(90, -30, 30);
    straight(7, 30);
    //armの角度を調整した方がいい
    arm_take_obj();
    straight(7, -30);
    arm_reset_A();
    turn(90, 30, -30);
    straight(28.5, 30);
    turn(90, -30, 30);
    straight(7, 30);
    //armの角度を調整した方がいい
    arm_take_obj();
    straight(7, -30);
    arm_reset_A();
    turn(90, -30, 30);
    straight(14, 30);
    turn(90, 30, -30);
    straight(7, 30);
    //armの角度を調整した方がいい
    arm_take_obj();
    straight(7, -30);
    arm_reset_A();
    turn(180, -30, 30);


    stopping();

    //pattern3
    straight(21.5, 30);
    turn(90, -30, 30);
    straight(7, 30);
    //armの角度を調整した方がいい
    arm_take_obj();
    straight(7, -30);
    arm_reset_A();
    turn(90, -30, 30);
    straight(28.5, 30);
    straight(7, 30);
    //armの角度を調整した方がいい
    arm_take_obj();
    straight(7, -30);
    arm_reset_A();
    turn(100, -30, 0);
    turn(100, 0, -30);
    straight(7, 30);
    //armの角度を調整した方がいい
    arm_take_obj();
    straight(7, -30);
    arm_reset_A();
    turn(90, 30, -30);
    straight(7, 30);
    turn(90, 30, -30);


    stopping();

    //pattern4
    straight(7.5, -30);
    turn(90, -30, 30);
    straight(7, 30);
    //armの角度を調整した方がいい
    arm_take_obj();
    straight(7, -30);
    arm_reset_A();
    turn(90, 30, -30);
    straight(21, 30);
    turn(-30, 30, 90);
    straight(7, 30);
    //armの角度を調整した方がいい
    arm_take_obj();
    straight(7, -30);
    arm_reset_A();
    turn(100, 0, -30);
    turn(100, -30, 0);
    straight(22, 30);
    //armの角度を調整した方がいい
    arm_take_obj();
    straight(7, -30);
    arm_reset_A();
    turn(180, 30, -30);


    linetrace_cm(30, 0.35, 10);
    linetrace_color(20, 0.4, BOTH, COLOR_BLACK);
    linetrace_cm(30, 0.35, 20);
    linetrace_cm(15, 0.6, 5);
    //armで船掴む
    arm_take_ship();

    straight(20, -30);
    turn(90, -30, 30);
    linetrace_cm(12, 0.6, 10);
    linetrace_cm(40, 0.2, 35);
    linetrace_color(20, 0.6, BOTH, COLOR_BLACK);
    //arm開く
    arm_reset_A();

    turn(180, 30, -30);
    straight(16.5, -30);
    //armおろしてオブジェクト下ろす
    turn(180, 30, 0);
    linetrace_cm(25, 0.4, 24);
    linetrace_cm(12, 0.6, 8);
    arm_take_obj();
    //armでオブジェクトとる
    turn(180, -30, 0);
    arm_reset_A();
    straight(98, 50);
    arm_take_ship();
    //armで船掴む
    turn(180, 0, 30);
    straight(14, 30);
    turn(180, 0, 30);
    linetrace_cm(20, 0.6, 15);
    linetrace_cm(30, 0.3, 135);
    linetrace_color(20, 0.5, BOTH, COLOR_BLACK);
    straight(17, -30);
    turn(35, 30, -30);
    straight(28.5, 30);
    turn(55, 30, -30);
    linetrace_cm(20, 0.6, 10);
    //arm戻す
    arm_reset_A();
    turn(180, 30, -30);
    linetrace_cm(30, 0.35, 55);


    battery = ev3_battery_voltage_mV();
    fprintf(bt, "BATTERY:%d", battery);


}
