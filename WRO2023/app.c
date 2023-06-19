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

/**
 * Define the connection ports of the sensors and motors.
 * By default, this application uses the following ports:
 * Touch sensor: Port 2
 * Color sensor: Port 3
 * Left motor:   Port B
 * Right motor:  Port C
 */
static const sensor_port_t  PortSensorColor2 = EV3_PORT_2;
static const sensor_port_t  PortSensorColor3 = EV3_PORT_3;
static const motor_port_t   PortMotorLeft   = EV3_PORT_B;
static const motor_port_t   PortMotorRight  = EV3_PORT_C;

uint8_t obj = 0;
uint8_t test = 0;
rgb_raw_t testrgb;
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

void straight(float cm, int power);
void turn(int lb_power, int rc_power, int angle);
void steering_color(colorid_t color_stop, int power, int steering);
void start();
void check_SIC();




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

void turn(int lb_power, int rc_power, int angle){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int lb_sign = lb_power / abs(lb_power);
    int rc_sign = rc_power / abs(rc_power);
    int now_right_angle = 0;
    int now_left_angle = 0;
    int maximum = 80;
    int points = 30;
    if (abs(lb_power) >= abs(rc_power)) maximum = abs(lb_power);
    if (abs(rc_power) > abs(lb_power)) maximum = abs(rc_power);
    float changing_power = 15;
    int goal_angle = angle*0.1531*ROBOT1CM;
    while (true) {
        now_left_angle = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_right_angle = abs(ev3_motor_get_counts(EV3_PORT_C));
        
        if (changing_power <= 15) changing_power = 15;
        if (lb_power == 0) {
            if (changing_power < maximum && goal_angle - (points*0.1531*ROBOT1CM) > now_right_angle) changing_power = changing_power + 0.007;
            if (goal_angle - (points*0.1531*ROBOT1CM) <= now_right_angle) changing_power = changing_power - 0.17;
            if (changing_power <= 20) changing_power = 20;
            if (goal_angle <= now_right_angle) break; 
            rc_power = changing_power*rc_sign;
            ev3_motor_set_power(EV3_PORT_C, rc_power);
        }
        if (rc_power == 0) {
            if (changing_power < maximum && goal_angle - (points*0.1531*ROBOT1CM) > now_left_angle) changing_power = changing_power + 0.007;
            if (goal_angle - (points*0.1531*ROBOT1CM) <= now_left_angle) changing_power = changing_power - 0.17;
            if (changing_power <= 20) changing_power = 20;
            if (goal_angle <= now_left_angle) break; 
            lb_power = -changing_power*lb_sign;
            ev3_motor_set_power(EV3_PORT_B, lb_power);

        }
        if (lb_power != 0 && rc_power != 0){
            if (changing_power < maximum && goal_angle - (points*0.1531*ROBOT1CM) > now_right_angle) changing_power = changing_power + 0.004;
            if (goal_angle - (points*0.1531*ROBOT1CM) <= now_right_angle) changing_power = changing_power - 0.1;
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

    int diff;
    int steering;
    while (true)
    {
        
        now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        now_angle_lb = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_C));
        average = (now_angle_lb + now_angle_rc) / 2;
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

        if (average >= ROBOT1CM*cm) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);

}

void linetrace_color(int power, int gain, port_t port, colorid_t color){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int lb_power;
    int rc_power;
    int now_reflect_2; 
    int now_reflect_3;
    int color_2;
    int color_3; 

    int diff;
    int steering;
    while (true)
    {
        now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        color_2 = ev3_color_sensor_get_color(EV3_PORT_2);
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
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

        if (port == RIGHT && color_3 == color)break;
        if (port == LEFT && color_2 == color)break;
        if (port == BOTH && color_2 == color && color_3 == color)break;
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
        obj_distance = ev3_ultrasonic_sensor_get_distance(EV3_PORT_4);
        check_type = 1;
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
                if (judgement > 100 || blue > 40 || green > 40 || red > 40 || (red - green - blue > 12) || (blue - red > 15)) {
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
        if (obj_distance < 8) {
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
                    if (judgement > 100 || blue > 35 || green > 35 || red > 35 || (h < 200 && h > 100) || (blue - red > 14) || (red - blue > 10) ) {
                        location[num] = PERSON;
                        if (red - blue > 19 || (h > 10 && h < 50 && judgement > 20)) location[num] = FIRE;    
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
}

void sensor_check(sensor_port_t sensor){
    colorid_t color = ev3_color_sensor_get_rgb_raw(sensor_port_t 	port,rgb_raw_t * 	val )		
}





void stopping(){
    while(ev3_button_is_pressed(ENTER_BUTTON) == false) {}    
    tslp_tsk(2000*MSEC);
}

void start(){

}

void main_task(intptr_t unused) {
    /* Register button handlers */
    ev3_button_set_on_clicked(BACK_BUTTON, &button_clicked_handler, BACK_BUTTON);

    /* Configure motors */
    ev3_motor_config(PortMotorLeft, LARGE_MOTOR);
    ev3_motor_config(PortMotorRight, LARGE_MOTOR);

    /* Configure sensors */
    ev3_sensor_config(PortSensorColor2, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor3, COLOR_SENSOR);

    /* ここからコーディング */
    stopping();
    steering_reflect(30, 20, 0);
    linetrace_cm(30, 0.7, 17);

    //ここで色を読む
    straight(5, 30);
    //ここで色を読む
    straight(14, 30);
    straight(10, -30);
    turn(30, -30, 90);
    linetrace_cm(30, 0.3, 40);
    linetrace_color(30, 0.5, BOTH, COLOR_BLACK);
    straight(5.5, 30);
    turn(-30, 30, 90);
    //ev3_motor_rotate();
    linetrace_cm(30, 0.7, 26.5);
    turn(30, 0, 180);
    straight(6, -30);
    //ここで色を読む
    straight(7, -30);
    //ここで色を読む
    straight(7, -30);
    //ここで色を読む
    straight(7, -30);
    //ここで色を読む
    //何やかんやあってオブジェクトとりました
    stopping();
    linetrace_color(30, 0.7, BOTH, COLOR_BLACK);
    linetrace_cm(30, 0.7, 24.5);
    //armで船掴む
    straight(18, -30);
    turn(-30, 30, 90);
    linetrace_cm(50, 0.7, 45);
    linetrace_color(30, 0.7, BOTH, COLOR_BLACK);
    //arm開く
    turn(30, -30, 180);
    straight(13, -30);
    //armおろしてオブジェクト下ろす
    turn(30, 0, 180);
    linetrace_cm(30, 0.7, 30);
    //armでオブジェクトとる
    turn(-30, 0, 180);
    straight(98, 50);
    //armで船掴む
    turn(0, 30, 180);
    straight(12, 30);
    turn(0, 30, 180);
    linetrace_cm(50, 0.7, 150);
    linetrace_color(30, 0.6, BOTH, COLOR_BLACK);
    straight(17, -30);
    turn(30, -30, 45);
    straight(31, 30);
    turn(30, -30, 45);
    linetrace_cm(30, 0.7, 10);
    //arm戻す
    turn(30, -30, 180);





}
