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
#define ROBOT1CM (18.48)
#define TURN (0.17)

/**
 * Define the connection ports of the sensors and motors.
 * By default, this application uses the following ports:
 * Touch sensor: Port 2
 * Color sensor: Port 3
 * Left motor:   Port B
 * Right motor:  Port C
 */
static const sensor_port_t  PortUltrasonic = EV3_PORT_1;
static const sensor_port_t  PortSensorColor2 = EV3_PORT_2;
static const sensor_port_t  PortSensorColor3 = EV3_PORT_3;
static const sensor_port_t  PortSensorColor4 = EV3_PORT_4;
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
int power = 50;
int steer;

int distance;


/*ライントレース用の変数の定義*/
int reflect, reflect2, reflect3;
float p, i, d, d2;

int start;
colorid_t color;

/*タイムアウトタスクの変数の定義*/
int TIME;
SYSTIM NOWTIME;
SYSTIM STARTTIME;

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

int location[12];


rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)

int red=0;
int green=0;
int blue=0;


void steering(float length, int power, int steering){
    int true_steering = 0;
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    	
    if(steering > 0) {
        (void)ev3_motor_rotate(EV3_PORT_B, length*ROBOT1CM, -power, false);
        (void)ev3_motor_rotate(EV3_PORT_C, length*ROBOT1CM, power-(power*steering/50), false);
    }
    else {
        (void)ev3_motor_rotate(EV3_PORT_B, length*ROBOT1CM, -(power+(power*steering/50)), false);
        (void)ev3_motor_rotate(EV3_PORT_C, length*ROBOT1CM, power, false);
    }
    while(true_steering < 1){
        if (ev3_motor_get_counts(EV3_PORT_B) > length*ROBOT1CM) {
            (void)ev3_motor_stop(EV3_PORT_B, true);
            (void)ev3_motor_stop(EV3_PORT_C, true);
            true_steering = true_steering + 1;
        }
        else if (ev3_motor_get_counts(EV3_PORT_C) > length*ROBOT1CM) {
            (void)ev3_motor_stop(EV3_PORT_B, true);
            (void)ev3_motor_stop(EV3_PORT_C, true);
            true_steering = true_steering + 1;
        }
    }
}

void tank_turn(float angle, int power_L, int power_R){
    if (power_R == 0) {
        (void)ev3_motor_rotate(EV3_PORT_B, angle*TURN*ROBOT1CM, (int16_t)-power_L, true);
    } 
    if (power_L == 0) {
        (void)ev3_motor_rotate(EV3_PORT_C, angle*TURN*ROBOT1CM, (int16_t)power_R, true);
    }
    if (power_L != 0 && power_R != 0) {
        (void)ev3_motor_rotate(EV3_PORT_B, angle*TURN*ROBOT1CM, (int16_t)-power_L, false);
        (void)ev3_motor_rotate(EV3_PORT_C, angle*TURN*ROBOT1CM, (int16_t)power_R, true);
        ev3_motor_stop(EV3_PORT_B, false);
        ev3_motor_stop(EV3_PORT_C, true);
    }
}

void tank_turn_color(int power_L, int power_R){

    colorid_t color;
    ev3_motor_set_power(EV3_PORT_C, power_R);
    ev3_motor_set_power(EV3_PORT_B, -power_L);
    while(color != COLOR_WHITE) {
        if(power_L > 0) {
            color = ev3_color_sensor_get_color(EV3_PORT_2);
        } 
        else {
            color = ev3_color_sensor_get_color(EV3_PORT_3);
        }
    }
    while(color != COLOR_BLACK) {
        if(power_L > 0) {
            color = ev3_color_sensor_get_color(EV3_PORT_2);
        } 
        else {
            color = ev3_color_sensor_get_color(EV3_PORT_3);
        }
    }
    (void)ev3_motor_stop(EV3_PORT_B, false);
    (void)ev3_motor_stop(EV3_PORT_C, true);    
}

void steering_color(colorid_t color_stop, int power, int steering){
    colorid_t color;
    if(steering > 0) {
        ev3_motor_set_power(EV3_PORT_B, -power);
        ev3_motor_set_power(EV3_PORT_C, power-(power*steering/50));
    }
    else {
        ev3_motor_set_power(EV3_PORT_B, -(power+(power*steering/50)));
        ev3_motor_set_power(EV3_PORT_C, power);
    }
    while (color_stop != color) {
        color = ev3_color_sensor_get_color(EV3_PORT_3);
    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);    
}

void steering_time(int time_stop_4d, int power, int steering){
    if(steering > 0) {
        ev3_motor_set_power(EV3_PORT_B, -power);
        ev3_motor_set_power(EV3_PORT_C, power-(power*steering/50));
    }
    else {
        ev3_motor_set_power(EV3_PORT_B, -power);
        ev3_motor_set_power(EV3_PORT_C, power-(power*steering/50));
    }
    tslp_tsk(time_stop_4d * MSEC);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void linetrace_color(sensortype_t type, colorid_t color_stop, int power){
    colorid_t color2 = COLOR_NONE;
    colorid_t color3 = COLOR_NONE;
    int reflect_stop = 0;
    if (color_stop == COLOR_BLACK) {
        reflect_stop = 15;
    }
    while (true) {
        color2 = ev3_color_sensor_get_color(EV3_PORT_2);
        color3 = ev3_color_sensor_get_color(EV3_PORT_3);
        reflect2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        reflect3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        reflect = reflect2 - reflect3;
        p = reflect;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  p * P_GEIN;    
        if(color_stop != COLOR_BLACK && color2 == color_stop && color3 == color_stop && type == BOTH) break;
        if(color_stop != COLOR_BLACK && color3 == color_stop && type == RIGHT) break;
        if(color_stop != COLOR_BLACK && color2 == color_stop && type == LEFT) break;
        if(color_stop == COLOR_BLACK && reflect2 < reflect_stop && reflect3 < reflect_stop && type == BOTH) break;
        if(color_stop == COLOR_BLACK && reflect3 < reflect_stop && type == RIGHT) break;
        if(color_stop == COLOR_BLACK && reflect2 < reflect_stop && type == LEFT) break;        
        if(steer > 0) {
            ev3_motor_set_power(EV3_PORT_B, -power);
            ev3_motor_set_power(EV3_PORT_C, power-(power*steer/50));
        }
        else {
            ev3_motor_set_power(EV3_PORT_B, -power);
            ev3_motor_set_power(EV3_PORT_C, power-(power*steer/50));
        }     
    }  
    ev3_speaker_play_tone(NOTE_AS5, 100);
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
    
}

void linetrace_reflect(sensortype_t type, int reflect_stop, int power){

    while (true) {

        reflect2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        reflect3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        reflect = reflect2 - reflect3;
        p = reflect;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  p * P_GEIN;    
        if(reflect_stop > reflect2 && reflect_stop > reflect3 && type == BOTH) break;
        if(reflect_stop > reflect3 && type == RIGHT) break;
        if(reflect_stop > reflect2 && type == LEFT) break;

        if(steer > 0) {
            ev3_motor_set_power(EV3_PORT_B, -power);
            ev3_motor_set_power(EV3_PORT_C, power-(power*steer/50));
        }
        else {
            ev3_motor_set_power(EV3_PORT_B, -power);
            ev3_motor_set_power(EV3_PORT_C, power-(power*steer/50));
        }     
    }  
    ev3_speaker_play_tone(NOTE_AS5, 100);
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
    
}


void linetrace_length(float length, int power){
    ev3_motor_reset_counts(EV3_PORT_C);
    while (length * ROBOT1CM > kakudo_C) {
        kakudo_C = ev3_motor_get_counts(EV3_PORT_C);
        reflect2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        reflect3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        reflect = reflect2 - reflect3;
        p = reflect;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  p * P_GEIN; 
        if(steer > 0) {
            ev3_motor_set_power(EV3_PORT_B, -power);
            ev3_motor_set_power(EV3_PORT_C, power-(power*steer/50));
        }
        else {
            ev3_motor_set_power(EV3_PORT_B, -power);
            ev3_motor_set_power(EV3_PORT_C, power-(power*steer/50));
        }

    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
}

void sensor_check(motor_port_t port) {
    switch (port) {
    case EV3_PORT_1:
        /* code */
        break;
    case EV3_PORT_2:
        /* code */
        break;
    case EV3_PORT_3:
        /* code */
        break;    
    case EV3_PORT_4:
        /* code */
        break;
    default:
        break;
    }
}

void map_check(int num) {

    uint8_t obj_under;
    ht_nxt_color_sensor_measure_color(EV3_PORT_4, &obj_under);
    switch (obj_under){
        case 2:
        case 3:
            location[num] = ADULT;
            ev3_speaker_play_tone(NOTE_E5, 100);
            tslp_tsk(200*1000);
            ev3_speaker_play_tone(NOTE_E5, 100);
            tslp_tsk(200*1000);
            ev3_speaker_play_tone(NOTE_E5, 100);
            tslp_tsk(200*1000);
            ev3_speaker_play_tone(NOTE_E5, 100);
            break;
        case 4:
            location[num] = CHILD;
            ev3_speaker_play_tone(NOTE_C5, 100);
            tslp_tsk(200*1000);
            ev3_speaker_play_tone(NOTE_C5, 100);
            tslp_tsk(200*1000);
            ev3_speaker_play_tone(NOTE_C5, 100);
            break;
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
            location[num] = FIRE;
            ev3_speaker_play_tone(NOTE_D5, 100);
            tslp_tsk(200*1000);
            ev3_speaker_play_tone(NOTE_D5, 100);
            tslp_tsk(200*1000);
            ev3_speaker_play_tone(NOTE_D5, 100);
            tslp_tsk(200*1000);
            ev3_speaker_play_tone(NOTE_D5, 100);
            tslp_tsk(200*1000);
            ev3_speaker_play_tone(NOTE_D5, 100);
            break;
        case 0:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
            distance = ev3_ultrasonic_sensor_get_distance(EV3_PORT_1);
            if (distance < 6) {
                location[num] = CHEMICAL;
                ev3_speaker_play_tone(NOTE_C4, 100);
                tslp_tsk(100*1000);
                ev3_speaker_play_tone(NOTE_C4, 100);
            }
            else {
                location[num] = NOTHING;
                ev3_speaker_play_tone(NOTE_B6, 100);
            }
            break;
        default:
            ev3_speaker_play_tone(NOTE_D6, 100);
            tslp_tsk(200*1000);
            ev3_speaker_play_tone(NOTE_D6, 100);
            tslp_tsk(200*1000);
            ev3_speaker_play_tone(NOTE_D6, 100);
            tslp_tsk(200*1000);
            ev3_speaker_play_tone(NOTE_D6, 100);
            tslp_tsk(200*1000);
            ev3_speaker_play_tone(NOTE_D6, 100);
            tslp_tsk(200*1000);
            ev3_speaker_play_tone(NOTE_D6, 100);
            break;
    } 
}

void timeout_task(intptr_t unused) {
    SYSTIM nowtime, time;   
    char str[64];
    get_tim(&nowtime);
    time = nowtime - STARTTIME;
    sprintf(str, "TIME:%ld", time);
    ev3_lcd_draw_string(str, 1, 1);
    
}

void main_task(intptr_t unused) {

    
    /* Register button handlers */
    ev3_button_set_on_clicked(BACK_BUTTON, &button_clicked_handler, BACK_BUTTON);

    /* Configure motors */
    ev3_motor_config(PortMotorLeft, LARGE_MOTOR);
    ev3_motor_config(PortMotorRight, LARGE_MOTOR);

    /* Configure sensors */
    ev3_sensor_config(PortUltrasonic, ULTRASONIC_SENSOR);
    ev3_sensor_config(PortSensorColor2, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor3, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor4, HT_NXT_COLOR_SENSOR);

    get_tim(&STARTTIME);
    ev3_lcd_set_font(EV3_FONT_SMALL);

    ev3_color_sensor_get_color(EV3_PORT_3);
    ev3_color_sensor_get_color(EV3_PORT_2);
    ev3_color_sensor_get_color(EV3_PORT_1);

   (void)sta_cyc(TIMEOUT_CYC);

    /*ここからコーディング */

    /*スタートの分岐チェック*/
    distance = ev3_ultrasonic_sensor_get_distance(EV3_PORT_1); 
    if (distance > 30) {
        start = 1;
    }
    else {
        ev3_speaker_play_tone(NOTE_AS5, 100);
        start = 2;
    }

    /*スタート*/
    switch(start){
        case 1:
            steering(80, 60, 0);
            steering_color(COLOR_WHITE, 30, 0);
            steering_color(COLOR_BLACK, 15, 0);
            steering(11.5, 20, 0);
            tank_turn(75, 25, -25);
            tank_turn_color(25, -25);
            ev3_speaker_play_tone(NOTE_AS5, 100);

            linetrace_length(10, 15);
            linetrace_color(BOTH, COLOR_BLACK, 20);
            ev3_speaker_play_tone(NOTE_AS5, 100);
            linetrace_length(15, 40);
            linetrace_color(LEFT, COLOR_BLACK, 20);
            ev3_speaker_play_tone(NOTE_AS5, 100);
            steering(11.5, 20, 0);
            tank_turn(75, -25, 25);
            tank_turn_color(-25, 25);
            linetrace_color(BOTH, COLOR_BLUE, 20);
            break;
        case 2:
            tank_turn(160, 0, 35);
            ev3_speaker_play_tone(NOTE_AS5, 100);
            tank_turn(160, 35, 0);
            ev3_speaker_play_tone(NOTE_AS5, 100);
            steering(50, 60, 0);
            steering_color(COLOR_WHITE, 30, 0);
            steering_color(COLOR_BLACK, 15, 0);
            steering(22, 30, 0);
            tank_turn(180, 0, -35);
            steering(5, -25, 0);
            linetrace_color(LEFT, COLOR_BLACK, 20);
            linetrace_length(11.5, 25);
            tank_turn(75, -30, 30);
            tank_turn_color(-25, 25);
            linetrace_color(BOTH, COLOR_BLUE, 20);
            break;
    }
    /*blue*/
    steering(5.5, 25, 0);
    map_check(0);
    steering(23.7, 30, 0);
    tank_turn(90, -30, 30);
    steering_time(1500, -50, 0);
    steering_time(500, -10, 0);
    tslp_tsk(10 * MSEC);
    steering(5, 30, 0);
    steering(3, -30, 0);
    tslp_tsk(10 * MSEC);
    tank_turn(180, 35, 0);
    steering(4, 30, 0);
    map_check(1);
    /*green*/
    steering(11, 30, 0);
    map_check(2);
    steering(35, 30, 0);
    map_check(3);
    /*yellow*/
    steering(16, -30, 0);
    tank_turn(180, 0, 25);
    steering_time(1500, -50, 0);
    steering_time(500, -10, 0);
    steering_color(COLOR_YELLOW, 30, 0);
    steering(7.2, 30, 0);
    map_check(4);
    /*red*/
    steering_color(COLOR_RED, 30, 0);
    steering(7.2, 30, 0);
    map_check(10);

    /*yellow*/

    tank_turn(180, -30, 0);
    steering_time(500, 30, 0);
    steering_time(1500, -50, 0);
    steering_time(500, -10, 0);
    tank_turn(98, 0, 25);
    tank_turn(95, 25, 0);
    steering(10.7, 30, 0);
    map_check(7);
    /*white*/
    steering_color(COLOR_WHITE, 20, 0);
    steering(7.8, 30, 0);
    map_check(6);
    steering(36.9, 30, 0);
    map_check(5);
    linetrace_color(BOTH, COLOR_BLACK, 20);
    tank_turn(180, 30, 0);
    linetrace_color(RIGHT, COLOR_BLACK, 20);
    tank_turn(180, 30, 0);
    linetrace_reflect(BOTH, 20, 30);
    /*brown*/
    steering(5.5, 25, 0);
    map_check(8);
    steering(36.8, 30, 0);
    map_check(9);

    location[11] = 16 - (location[0] + location[1] + location[2] + location[3] + location[4] + location[5] + location[6] + location[7] + location[8] + location[9] + location[10]);

    /*chemical*/
    steering(40, -30, 0);
    tank_turn(180, 25, -25);
    

    while(1) {}





}
