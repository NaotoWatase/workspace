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

float robot1cm = 18.48;
float turn = 0.1326;

 
void straight(float length, int power){
    (void)ev3_motor_rotate(EV3_PORT_B, length*robot1cm, -power, false);
    (void)ev3_motor_rotate(EV3_PORT_C, length*robot1cm, power, true);
}

void tank_turn(float angle, int power_L, int power_R){
    (void)ev3_motor_rotate(EV3_PORT_B, angle*turn*robot1cm, (int16_t)-power_L, false);
    (void)ev3_motor_rotate(EV3_PORT_C, angle*turn*robot1cm, (int16_t)power_R, true);
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

    float length=100000.0;
    int power=40;
    straight(length, power);

    

    

}   