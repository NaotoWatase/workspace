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
static const sensor_port_t  PortSensorTouch = EV3_PORT_2;
static const sensor_port_t  PortSensorColor = EV3_PORT_3;
static const motor_port_t   PortMotorLeft   = EV3_PORT_B;
static const motor_port_t   PortMotorRight  = EV3_PORT_C;

void straight(int power, int steering, float cm);
void turn(int lb_power, int rc_power, int angle);

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

void straight(int power, int steering, float cm){
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
    float changing_power = 5;
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
        if (changing_power <= 5) changing_power = 5;
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

void test_turn(int lb_power, int rc_power, int angle){
    ev3_motor_rotate(EV3_PORT_B, 90*0.1532*ROBOT1CM, 30, false);
    ev3_motor_rotate(EV3_PORT_C, 90*0.1532*ROBOT1CM, 30, true);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}





void stopping(){
    while(ev3_button_is_pressed(ENTER_BUTTON) == false) {}    
    tslp_tsk(2000*MSEC);
}

void main_task(intptr_t unused) {
    /* Register button handlers */
    ev3_button_set_on_clicked(BACK_BUTTON, &button_clicked_handler, BACK_BUTTON);

    /* Configure motors */
    ev3_motor_config(PortMotorLeft, LARGE_MOTOR);
    ev3_motor_config(PortMotorRight, LARGE_MOTOR);

    /* Configure sensors */
    ev3_sensor_config(PortSensorTouch, TOUCH_SENSOR);
    ev3_sensor_config(PortSensorColor, COLOR_SENSOR);

    /* ここからコーディング */
    while (true)
    {
        tslp_tsk(1000*MSEC);



        turn(0, 80, 180);
        tslp_tsk(100*MSEC);
        turn(0, -80, 180);
        tslp_tsk(100*MSEC);
        turn(80, 0, 180);
        tslp_tsk(100*MSEC);
        turn(-80, 0, 180);
        turn(0, 80, 180);
        tslp_tsk(100*MSEC);
        turn(0, -80, 180);
        tslp_tsk(100*MSEC);
        turn(80, 0, 180);
        tslp_tsk(100*MSEC);
        turn(-80, 0, 180);



        stopping();
        straight(100, 0, 30);
        straight(100, 0, 30);
        straight(100, 0, 30);
        straight(-100, 0, 30);
        straight(-100, 0, 30);
        straight(-100, 0, 30);
    }
}
