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


#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

#define MSEC (1000)

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

static void button_clicked_handler(intptr_t button) {
    switch(button) {
    case BACK_BUTTON:
#if !defined(BUILD_MODULE)
        syslog(LOG_NOTICE, "Back button clicked.");
#endif
        break;
    }
}

int kakudo_C; 
float robot1cm = 18.48;
float turn = 0.1326;
colorid_t color;
int reflect2;
int reflect3;
int reflect;
int steer;
int power = 50;
float p;
float i;
float d;
float d2;
int start;

void steering(float length, int power, float steering){
    if(steering > 0) {
        (void)ev3_motor_rotate(EV3_PORT_B, length*robot1cm, -power, false);
        (void)ev3_motor_rotate(EV3_PORT_C, length*robot1cm, power-(power*steering/50), true);
    }
    else {
        (void)ev3_motor_rotate(EV3_PORT_B, length*robot1cm, -(power+(power*steering/50)), false);
        (void)ev3_motor_rotate(EV3_PORT_C, length*robot1cm, power, true);
    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
}


void tank_turn(float angle, int power_L, int power_R){
    (void)ev3_motor_rotate(EV3_PORT_B, angle*turn*robot1cm, (int16_t)-power_L, false);
    (void)ev3_motor_rotate(EV3_PORT_C, angle*turn*robot1cm, (int16_t)power_R, true);
}

void straight_color(colorid_t color_stop, int power){
    while (color_stop != color) {
        color = ev3_color_sensor_get_color(EV3_PORT_3);
        (void)ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, 0);
    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);    
}

void linetrace_color(colorid_t color_stop, int power){
    while (color_stop != color) {
        color = ev3_color_sensor_get_color(EV3_PORT_3);
        reflect2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        reflect3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        reflect = reflect2 - reflect3;
        p = reflect;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  p * P_GEIN + i * I_GEIN + d * D_GEIN;
        (void)ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
}

void linetrace_length(float length, int power){
    ev3_motor_reset_counts(EV3_PORT_C);
    while (length * robot1cm > kakudo_C) {
        kakudo_C = ev3_motor_get_counts(EV3_PORT_C);
        reflect2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        reflect3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        reflect = reflect2 - reflect3;
        p = reflect;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  p * P_GEIN + i * I_GEIN + d * D_GEIN;
        (void)ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
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

    /*ここからコーディング */
    char str[64];
    SYSTIM nowtime;
    ev3_lcd_set_font(EV3_FONT_SMALL);
    while (1) {
        get_tim(&nowtime);
        sprintf(str, "TIME:%ld", nowtime);
        ev3_lcd_draw_string(str, 1, 0);
        tslp_tsk(1000*MSEC);
    }
        
    


}