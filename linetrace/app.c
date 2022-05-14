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

#define THRESHOLD 35

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

static void button_clicked_handler(intptr_t button) {
    switch(button) {
    case BACK_BUTTON:
#if !defined(BUILD_MODULE)
        syslog(LOG_NOTICE, "Back button clicked.");
#endif
        break;
    }
}

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
 
void straight(float length, int power){
    (void)ev3_motor_rotate(EV3_PORT_B, length*robot1cm, (int16_t)power, false);
    (void)ev3_motor_rotate(EV3_PORT_C, length*robot1cm, (int16_t)power, true);
}

void tank_turn(float angle, int power_L, int power_R){
    (void)ev3_motor_rotate(EV3_PORT_B, angle*turn*robot1cm, (int16_t)power_L, false);
    (void)ev3_motor_rotate(EV3_PORT_C, angle*turn*robot1cm, (int16_t)power_R, true);
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

    /*ここからコーディング */

    while (true) {
        color = ev3_color_sensor_get_color(EV3_PORT_3);
        reflect2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        reflect3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        reflect = reflect2 - reflect3;
        p = reflect;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  p * 0.5 + i * 0.002 + d * 0.4;
        (void)ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);

    

} 