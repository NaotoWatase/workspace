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

void main_task(intptr_t unused) {
    int16_t distance = (int16_t )30;
    int32_t temp;
    int32_t rotateAngleLeft;
    int32_t rotateAngleRight;

    /* Register button handlers */
    ev3_button_set_on_clicked(BACK_BUTTON, &button_clicked_handler, BACK_BUTTON);

    /* Configure motors */
    ev3_motor_config(PortMotorLeft, LARGE_MOTOR);
    ev3_motor_config(PortMotorRight, LARGE_MOTOR);

    /* Configure sensors */
    ev3_sensor_config(PortSensorTouch, TOUCH_SENSOR);
    ev3_sensor_config(PortSensorColor, COLOR_SENSOR);

    /* ここからコーディング */
    temp = (int32_t)distance * 10 * 100 * 360; 	/* distance をint32_t型に変換している */
    rotateAngleLeft = temp  / 56 / 314;
    rotateAngleRight = rotateAngleLeft;	/* 直進なので、左右で同じ回転角度 */

    (void)ev3_motor_rotate(EV3_PORT_B, rotateAngleLeft, (uint32_t)100U, false);
    (void)ev3_motor_rotate(EV3_PORT_C, rotateAngleRight, (uint32_t)100U, true);
}
