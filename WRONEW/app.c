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


#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

#define MSEC (1000)
#define ROBOT1CM (18.48)
#define TURN (0.18)

/**
 * Define the connection ports of the sensors and motors.
 * By default, this application uses the following ports:
 * Touch sensor: Port 2
 * Color sensor: Port 3
 * Left motor:   Port B
 * Right motor:  Port C
 */
/*static const sensor_port_t  PortUltrasonic = EV3_PORT_1;
static const sensor_port_t  PortSensorColor2 = EV3_PORT_2;
static const sensor_port_t  PortSensorColor3 = EV3_PORT_3;
static const sensor_port_t  PortSensorColor4 = EV3_PORT_4;
static const motor_port_t   PortMotorWater   = EV3_PORT_D;
static const motor_port_t   PortMotorLeft   = EV3_PORT_B;
static const motor_port_t   PortMotorRight  = EV3_PORT_C;
static const motor_port_t   PortMotorArm  = EV3_PORT_A;*/
static const motor_port_t   PortMotorLeft   = EV3_PORT_B;
static const motor_port_t   PortMotorRight  = EV3_PORT_C;
static const motor_port_t   PortMotorArm  = EV3_PORT_A;
static const sensor_port_t PortSensorGYRO = EV3_PORT_4;


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

int kakudo_C; 
int power = 50;
float steer;
float distance;

int brown;

int map [6] = {0,0,0,0,0,0};
int start;


/*ライントレース終了条件の変数*/
typedef enum sensorType {
    RIGHT,
    LEFT,
    BOTH
} sensortype_t ;

/*mapのデータ変数*/
typedef enum object {  
    CHILD = 3,
    ADULT = 2,
    FIRE = 5,
    CHEMICAL = 1,
    NOTHING = 0
} object_t ;

typedef enum UpDown {
    up = 1,
    down = 0
} arm_t ;

int location[12] = {0,0,0,0,0,0,0,0,0,0,0,0};


rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)

int red=0;
int green=0;
int blue=0;


void newsteering(int power, int cm) {
    int set_power = -power;
    int p;
    int gyro;
    int left;
    int right;
    int difference;
    float accele_length = cm / 10 * 1;
    float maxspeed_length = cm / 10 * 9;
    float decele_length = cm;
    
    
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
        /*if (power < 20 && power >= 0 && set_power > 0) power = 20;
        if (power > -20 && power <= 0 && set_power < 0) power = -20;*/
        if (power > -20 && set_power < 0) power = -20;
        if (power < 20 && set_power > 0) power = 20;
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
        if (decele_length * ROBOT1CM <= left) break;
    }
    ev3_speaker_play_tone(NOTE_A4, 200);
    tslp_tsk(1000);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void tank_turn(float angle, int power_L, int power_R){
    if (power_R == 0) {
        (void)ev3_motor_rotate(EV3_PORT_B, angle*TURN*ROBOT1CM, (int16_t)-power_L, true);
    } 
    if (power_L == 0) {
        (void)ev3_motor_rotate(EV3_PORT_C, angle*TURN*ROBOT1CM, (int16_t)-power_R, true);
    }
    if (power_L != 0 && power_R != 0) {
        (void)ev3_motor_rotate(EV3_PORT_B, angle*TURN*ROBOT1CM, (int16_t)-power_L, false);
        (void)ev3_motor_rotate(EV3_PORT_C, angle*TURN*ROBOT1CM, (int16_t)-power_R, true);
        ev3_motor_stop(EV3_PORT_B, true);
        ev3_motor_stop(EV3_PORT_C, true);
    }
}

void p_turn(float angle, int left_motor, int right_motor){
    int turn_check = 0;
    int gyro = 0;
    int power;
    ev3_gyro_sensor_reset(EV3_PORT_4);
    while (true) {
        gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        gyro = abs(gyro);
        power = (angle - gyro) * 1;
        if(power > 65) power = 65;
        if (gyro > angle / 10 * 2) turn_check = turn_check + 1;

        if (left_motor == 0) {
            (void)ev3_motor_set_power(EV3_PORT_C, -power * right_motor); 
        }
        else if (right_motor == 0) {
            (void)ev3_motor_set_power(EV3_PORT_B, -power * left_motor);
        } 
        else {
            (void)ev3_motor_set_power(EV3_PORT_B, -power * left_motor);
            (void)ev3_motor_set_power(EV3_PORT_C, -power * right_motor); 
        }
        if (turn_check > 50) break;
    }
    while (true) {
        gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        gyro = abs(gyro);
        power = (angle - gyro) * 1;
        if(power > 65) power = 65;
        if (angle - gyro == 0) turn_check = turn_check + 1;

        if (left_motor == 0) {
            (void)ev3_motor_set_power(EV3_PORT_C, -power * right_motor); 
        }
        else if (right_motor == 0) {
            (void)ev3_motor_set_power(EV3_PORT_B, -power * left_motor);
        } 
        else {
            (void)ev3_motor_set_power(EV3_PORT_B, -power * left_motor);
            (void)ev3_motor_set_power(EV3_PORT_C, -power * right_motor); 
        }
        if (turn_check > 50) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}
    






void main_task(intptr_t unused) {

    
    /* Register button handlers */
    ev3_button_set_on_clicked(BACK_BUTTON, &button_clicked_handler, BACK_BUTTON);

    /* Configure motors */
    ev3_motor_config(PortMotorLeft, LARGE_MOTOR);
    ev3_motor_config(PortMotorRight, LARGE_MOTOR);
    ev3_motor_config(PortMotorArm, LARGE_MOTOR);



    /* Configure sensors */
    /*ev3_sensor_config(PortUltrasonic, ULTRASONIC_SENSOR);
    ev3_sensor_config(PortSensorColor2, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor3, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor4, HT_NXT_COLOR_SENSOR);*/
    ev3_sensor_config(PortSensorGYRO, GYRO_SENSOR);

    ev3_lcd_set_font(EV3_FONT_SMALL);

    /*ev3_button_is_pressed(ENTER_BUTTON);
    ev3_color_sensor_get_color(EV3_PORT_3);
    ev3_color_sensor_get_color(EV3_PORT_2);
    ev3_color_sensor_get_color(EV3_PORT_1);*/

    while(ev3_button_is_pressed(ENTER_BUTTON) == false) {}

    /*ここからコーディング */

    /*sensor
    0 = Black, 1 = Purple, 2 =PuBl, 3 = Brue, 4 = Green, 5 = GrYe,
    6 = Yellow, 7 = Orange, 8 = Red, 9 = PiRe, 10 = Pink, 
    11 = WhBl, 12 = WhGr, 13 = WhYe, 14 = WhOr, 15 = WhRe, 16 = WhPi,
    17 = White*/


    /*スタートの分岐チェック*/
    start = 2;
    tslp_tsk(1000*MSEC);
    p_turn(180, 1,-1);
    tslp_tsk(1000*MSEC);
    p_turn(180, -1,1);
    tslp_tsk(1000*MSEC);
    p_turn(360, -1,0);
    tslp_tsk(1000*MSEC);
    p_turn(360, 0,1);
    

    
    ev3_motor_rotate(EV3_PORT_A, 160,20, true);
    tslp_tsk(1000 * MSEC);
    ev3_motor_rotate(EV3_PORT_A, 160,-20, true);
    

}