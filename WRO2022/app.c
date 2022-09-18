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
#define TURN (0.165)

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
static const motor_port_t   PortMotorWater   = EV3_PORT_D;
static const motor_port_t   PortMotorLeft   = EV3_PORT_B;
static const motor_port_t   PortMotorRight  = EV3_PORT_C;
static const motor_port_t   PortMotorArm  = EV3_PORT_A;


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
int powers1;
int powers2;
int powers3;
int powers4;
int armc;
int chemical_check = 0;

int how_many = 16;

int music;
int kaisuu;

float distance;

int brown;

int map [6] = {0,0,0,0,0,0};

/*ライントレース用の変数の定義*/
int reflect, reflect2, reflect3;


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

typedef enum UpDown {
    up = 1,
    down = 0
} arm_t ;

int location[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
int location_sensor[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

uint8_t test = 0;
uint8_t obj = 0;

rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)

int red=0;
int green=0;
int blue=0;

int water_count = 0;

void steering(float length, int power, int steering, bool_t brake){
    int motor_count;
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);	
    if(steering > 0) {
        (void)ev3_motor_set_power(EV3_PORT_B, -power);
        (void)ev3_motor_set_power(EV3_PORT_C, power-(power*steering/50));
        while (true) {
            motor_count = ev3_motor_get_counts(EV3_PORT_C);
            if(length*ROBOT1CM <= motor_count && power > 0) break;
            if(length*ROBOT1CM <= -motor_count && power < 0) break;
        }
    }
    else {
        (void)ev3_motor_set_power(EV3_PORT_B, -(power+(power*steering/50)));
        (void)ev3_motor_set_power(EV3_PORT_C, power);
        while (true) {
            motor_count = ev3_motor_get_counts(EV3_PORT_C);
            if(length*ROBOT1CM <= motor_count && power > 0) break;
            if(length*ROBOT1CM <= -motor_count && power < 0) break;
        }
    }
    if(brake == true) {
        ev3_motor_stop(EV3_PORT_B, true);
        ev3_motor_stop(EV3_PORT_C, true);
    }
    /*if (abs(motor_count) > abs(motor_count_b)) {
        if(motor_count > 0) {
            ev3_motor_set_power(EV3_PORT_B, -8);
        }
        if(motor_count < 0) {
            ev3_motor_set_power(EV3_PORT_B, 8);
        }
    }
    if (abs(motor_count) < abs(motor_count_b)) {
        if(motor_count > 0) {
            ev3_motor_set_power(EV3_PORT_C, 8);
        }
        if(motor_count < 0) {
            ev3_motor_set_power(EV3_PORT_C, -8);
        }
    }
    while (true) {
        if (abs(motor_count) == abs(motor_count_b)) break;
    }
    */
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
        ev3_motor_stop(EV3_PORT_B, true);
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
    (void)ev3_motor_stop(EV3_PORT_B, true);
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

void steering_color_u(colorid_t color_stop, int power, int steering){
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
        color = ev3_color_sensor_get_color(EV3_PORT_2);
    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);    
}

void steering_color_b(colorid_t color_stop, int power, int steering){
    colorid_t color_2;
    colorid_t color_3;
    if(steering > 0) {
        ev3_motor_set_power(EV3_PORT_B, -power);
        ev3_motor_set_power(EV3_PORT_C, power-(power*steering/50));
    }
    else {
        ev3_motor_set_power(EV3_PORT_B, -(power+(power*steering/50)));
        ev3_motor_set_power(EV3_PORT_C, power);
    }
    while (true) {
        color_2 = ev3_color_sensor_get_color(EV3_PORT_2);
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_stop == color_2 && color_stop == color_3) break;
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
    float p = 0;
    float i = 0;
    float d = 0;
    float d2 = 0;
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
        i = reflect + i;
        d = reflect - d2; 
        d2 = reflect;
        steer =  p * P_GEIN + i * 0 + d * 0; 
        powers1 = -power;  
        powers2 = power-(power*steer/50);  
        if(color_stop != COLOR_BLACK && color2 == color_stop && color3 == color_stop && type == BOTH) break;
        if(color_stop != COLOR_BLACK && color3 == color_stop && type == RIGHT) break;
        if(color_stop != COLOR_BLACK && color2 == color_stop && type == LEFT) break;
        if(color_stop == COLOR_BLACK && reflect2 < reflect_stop && reflect3 < reflect_stop && type == BOTH) break;
        if(color_stop == COLOR_BLACK && reflect3 < reflect_stop && type == RIGHT) break;
        if(color_stop == COLOR_BLACK && reflect2 < reflect_stop && type == LEFT) break;        
        if(steer > 0) {
            ev3_motor_set_power(EV3_PORT_B, powers1);
            ev3_motor_set_power(EV3_PORT_C, powers2);
        }
        else {
            ev3_motor_set_power(EV3_PORT_B, powers1);
            ev3_motor_set_power(EV3_PORT_C, powers2);
        }     
    }  
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
    ev3_speaker_play_tone(NOTE_AS5, 100);
    tslp_tsk(100*MSEC);
    
}

void linetrace_reflect(sensortype_t type, int reflect_stop, int power){
    float p = 0;
    float i = 0;
    float d = 0;
    float d2 = 0;
    while (true) {
        reflect2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        reflect3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        reflect = reflect2 - reflect3;
        p = reflect;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  p * P_GEIN + i * 0 + d * 0;
        powers1 = -power;  
        powers2 = power-(power*steer/50);   
        if(reflect_stop > reflect2 && reflect_stop > reflect3 && type == BOTH) break;
        if(reflect_stop > reflect3 && type == RIGHT) break;
        if(reflect_stop > reflect2 && type == LEFT) break;

        if(steer > 0) {
            ev3_motor_set_power(EV3_PORT_B, powers1);
            ev3_motor_set_power(EV3_PORT_C, powers2);
        }
        else {
            ev3_motor_set_power(EV3_PORT_B, powers1);
            ev3_motor_set_power(EV3_PORT_C, powers2);
        }     
    }  
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
    ev3_speaker_play_tone(NOTE_AS5, 100);
    tslp_tsk(100*MSEC);
    
}


void walltrace_length(float length, int power, float dist){
    int PGEIN = 19;
    float p = 0;

    ev3_motor_reset_counts(EV3_PORT_C);
    while (true) {
        kakudo_C = ev3_motor_get_counts(EV3_PORT_C);
        float dist1 = ev3_ultrasonic_sensor_get_distance(EV3_PORT_1);
        if (dist1 > 50) {
            dist1 = 49; 
        } 
        p = dist1 - (dist + 1);

        steer =  p * PGEIN;
        powers1 = -power;  
        powers2 = power-(power*steer/50);
        if (length * ROBOT1CM < kakudo_C) break;
        if(steer > 0) {
            ev3_motor_set_power(EV3_PORT_B, -power);
            ev3_motor_set_power(EV3_PORT_C, power-(power*steer/50));
        }
        else {
            ev3_motor_set_power(EV3_PORT_B, -(power+(power*steer/50)));
            ev3_motor_set_power(EV3_PORT_C, power);
        }

    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
}

void walltrace_length_p(float length, int power, float dist, int pgein){
    int PGEIN = pgein;
    float p = 0;
    ev3_motor_reset_counts(EV3_PORT_C);
    while (true) {
        kakudo_C = ev3_motor_get_counts(EV3_PORT_C);
        float dist1 = ev3_ultrasonic_sensor_get_distance(EV3_PORT_1);
        if (dist1 > 50) {
            dist1 = 49; 
        } 
        p = dist1 - (dist + 1);
        steer =  p * PGEIN;
        powers1 = -power;  
        powers2 = power-(power*steer/50);
        if (length * ROBOT1CM < kakudo_C) break;
        if(steer > 0) {
            ev3_motor_set_power(EV3_PORT_B, -power);
            ev3_motor_set_power(EV3_PORT_C, power-(power*steer/50));
        }
        else {
            ev3_motor_set_power(EV3_PORT_B, -(power+(power*steer/50)));
            ev3_motor_set_power(EV3_PORT_C, power);
        }

    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
}

void linetrace_length(float length, int power){
    ev3_motor_reset_counts(EV3_PORT_C);
    float p = 0;
    float i = 0;
    float d = 0;
    float d2 = 0;
    while (true) {
        kakudo_C = ev3_motor_get_counts(EV3_PORT_C);
        reflect2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        reflect3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        reflect = reflect2 - reflect3;
        p = reflect;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  p * P_GEIN + i * 0 + d * 0; 
        powers1 = -power;  
        powers2 = power-(power*steer/50);
        if (length * ROBOT1CM < kakudo_C) break;
        if(steer > 0) {
            ev3_motor_set_power(EV3_PORT_B, powers1);
            ev3_motor_set_power(EV3_PORT_C, powers2);
        }
        else {
            ev3_motor_set_power(EV3_PORT_B, powers1);
            ev3_motor_set_power(EV3_PORT_C, powers2);
        }

    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
}

void water(int n) {
    ev3_motor_reset_counts(EV3_PORT_D);
    if (location[n] == FIRE) {
        ev3_motor_rotate(EV3_PORT_D, 35 + water_count, 20, true);
        tslp_tsk(100*MSEC);
        ev3_motor_rotate(EV3_PORT_D, 35 + water_count, -20, false);
        water_count = water_count + 80;
    }
}

void sensor_check(uint8_t num) {
    int ct = 0;
    while (ct < num) {
        ev3_speaker_play_tone(NOTE_C5, 100);
        tslp_tsk(120*1000);
        ct = ct + 1;
    }
}

void chemical_taker(int n){
    armc = 0;
    if(location[n] == CHEMICAL){
        if (chemical_check == 1) {
            steering(3, -30, 0, true);
            ev3_motor_rotate(EV3_PORT_A, 235, 15, true);
            steering(3, 30, 0, true);
        }
        if (chemical_check > 1) {
            steering(20, 30, 0, true);
            ev3_motor_rotate(EV3_PORT_A, 200, -20, true);
            ev3_motor_rotate(EV3_PORT_A, 25, 20, true);
            steering(3.7, -30, 0, true);
            ev3_motor_rotate(EV3_PORT_A, 140, 15, true);
            tslp_tsk(400*1000);
        }             
    }

}

void map_check(int num) {
    /*ev3_speaker_play_tone(NOTE_A5, 100);
    tslp_tsk(300*MSEC);*/

    while ( !ht_nxt_color_sensor_measure_color(EV3_PORT_4, &test) ) {
        ;
    }

    
    while ( !ht_nxt_color_sensor_measure_color(EV3_PORT_4, &obj) ) {
        ;
    }      

    distance = ev3_ultrasonic_sensor_get_distance(EV3_PORT_1);
    if (distance < 6) {
        location_sensor[num] = obj;
        switch (obj){
            case 1:
            case 2:
            case 3:
            case 11:
                location[num] = ADULT;
                break;
            case 4:
            case 13:
                location[num] = CHILD;
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
            case 12:
            case 14:
            case 16:
            case 17:
                    chemical_check = chemical_check + 1;
                    location[num] = CHEMICAL;
                    break; 
            default:
                break;
        } 
    }
    else {
        switch (obj){
            case 1:
            case 2:
            case 3:
                location[num] = ADULT;
            case 4:
                location[num] = CHILD;
                break;
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
            case 10:
                location[num] = FIRE;
                break;
            default:
                location[num] = NOTHING;
                break;
        }
    }
    /*ev3_speaker_play_tone(NOTE_A5, 100);
    tslp_tsk(400*MSEC);*/
}

void timeout_task(intptr_t unused) {
    SYSTIM nowtime, time;   
    char str[64];
    get_tim(&nowtime);
    time = nowtime - STARTTIME;
    sprintf(str, "TIME:%ld", time);
    ev3_lcd_draw_string(str, 1, 1);
    
}

void music_start(int times) {
    music = times;
}

void music_task(intptr_t unused) {
    /*ev3_speaker_play_tone(NOTE_C5, 100);
    tslp_tsk(200*1000);
    if (music > 0) {
        ev3_speaker_play_tone(NOTE_C4, 1000);
        music = music - 1;
    }    */
}

void arm(arm_t type) {
    switch (type)
    {
    case down:
        ev3_motor_rotate(EV3_PORT_A, 270, -40, true);
        tslp_tsk(400*MSEC);
        break;
    case up:
        ev3_motor_rotate(EV3_PORT_A, 270, 30, true);
        tslp_tsk(400*MSEC);
        break;
    }
    ev3_motor_stop(EV3_PORT_A, true);
}

void sp(int num) {
    if (location[num] == 3) {
        ev3_speaker_play_tone(NOTE_A5, 100);
        tslp_tsk(200*MSEC);
        if (chemical_check == 1) {
            ev3_motor_rotate(EV3_PORT_A, 35, 30, true);
            tslp_tsk(200*MSEC);
        }
        else {
            ev3_motor_rotate(EV3_PORT_A, 235, 30, true);
            tslp_tsk(400*MSEC);
            ev3_motor_rotate(EV3_PORT_A, 35, 30, true);
            tslp_tsk(500*MSEC);
            ev3_motor_rotate(EV3_PORT_A, 270, -30, true);
            tslp_tsk(400*MSEC);
        }
    }
}

void main_task(intptr_t unused) {

    
    /* Register button handlers */
    ev3_button_set_on_clicked(BACK_BUTTON, &button_clicked_handler, BACK_BUTTON);

    /* Configure motors */
    ev3_motor_config(PortMotorLeft, LARGE_MOTOR);
    ev3_motor_config(PortMotorRight, LARGE_MOTOR);
    ev3_motor_config(PortMotorWater, LARGE_MOTOR);
    ev3_motor_config(PortMotorArm, LARGE_MOTOR);


    /* Configure sensors */
    ev3_sensor_config(PortUltrasonic, ULTRASONIC_SENSOR);
    ev3_sensor_config(PortSensorColor2, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor3, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor4, HT_NXT_COLOR_SENSOR);

    get_tim(&STARTTIME);
    ev3_lcd_set_font(EV3_FONT_SMALL);

    ev3_button_is_pressed(ENTER_BUTTON);
    ev3_color_sensor_get_color(EV3_PORT_3);
    ev3_color_sensor_get_color(EV3_PORT_2);
    ev3_color_sensor_get_color(EV3_PORT_1);

    while(ev3_button_is_pressed(ENTER_BUTTON) == false) {}

    /*ここからコーディング */

    /*sensor
    0 = Black, 1 = Purple, 2 =PuBl, 3 = Brue, 4 = Green, 5 = GrYe,
    6 = Yellow, 7 = Orange, 8 = Red, 9 = PiRe, 10 = Pink, 
    11 = WhBl, 12 = WhGr, 13 = WhYe, 14 = WhOr, 15 = WhRe, 16 = WhPi,
    17 = White*/


    /*スタートの分岐チェック*/
    start = 2;

    /*スタート*/
    switch(start){
        case 1:
            steering(65, 60, 0, true);
            steering_color(COLOR_WHITE, 60, 0);
            steering_time(400, 25, 0);
            tank_turn(180, 0, 50);
            walltrace_length(75, 40, 8.5);
            walltrace_length_p(17, 20, 8.5, 22);
            steering_color(COLOR_BLACK, -30, 0);
            steering_color(COLOR_WHITE, -20, 0);
            linetrace_length(33.8, 30);
            break;
        case 2:
            tank_turn(85, 0, 40);
            tslp_tsk(100*MSEC);
            tank_turn(85, 40, 0);
            walltrace_length(75, 53, 8.5);
            walltrace_length_p(17, 20, 8.5, 22);
            steering_color(COLOR_BLACK, -30, 0);
            steering_color(COLOR_WHITE, -20, 0);
            ev3_speaker_play_tone(NOTE_A5, 100);
            tslp_tsk(400*MSEC);
            linetrace_length(33.8, 30);
            break;
    }
    
    /*blue*/
    tank_turn(46, 0, 30);
    tslp_tsk(100*1000);
    tank_turn(46, 30, 0);
    tslp_tsk(100*1000);

    map_check(0);
    chemical_taker(0);
    if(chemical_check == 2 && location[0] == CHEMICAL) {
        walltrace_length_p(4, 35, 9.5, 5);
    }
    else {
        steering(4.5, 35, 0, true);
        walltrace_length_p(16.7, 40, 9.5, 20);
    }
    steering(16, 35, 0, true);
    tslp_tsk(200 * MSEC);

    /*steering(22.9, 30, 0);
    tank_turn(90, -30, 30);
    steering_time(1000, -45, 0);
    steering_time(500, -15, 0);
    tslp_tsk(200 * MSEC);
    steering_time(250, 20, 0);
    tank_turn(180, 35, 0);
    steering(4, 30, 0);  */

    map_check(1);
    chemical_taker(1);
    if(location[0] == 3 || location[1] == 3) {
        steering(10, -35, 0, true);
        sp(0);
        sp(1);
        tslp_tsk(300 * MSEC);
        steering(9.5, 35, 0, true);
    }
    
    water(0);
    water(1);
    if(chemical_check == 2 && location[1] == CHEMICAL) {
        steering(3.5, -30, 0, true);
    }
    else {
        steering(11, 30, 0, true);
    }
    /*green*/
    
    map_check(2);
    chemical_taker(2);
    if(chemical_check == 2 && location[2] == CHEMICAL) {
        ev3_speaker_play_tone(NOTE_A4, 100);
        tslp_tsk(500 * MSEC);
        walltrace_length_p(11.5, 35, 9.5, 5);
    }
    else {
        steering(6.5, 35, 0, true);
        walltrace_length_p(20, 35, 9.5, 5);
    }
    
    /*steering(35, 25, 0);*/
    steering_time(800, 35, 0);
    map_check(3);
    if(location[2] == 3 || location[3] == 3) {
        steering(10, -35, 0, true);
        sp(2);
        sp(3);
        tslp_tsk(300 * MSEC);
        steering(9.5, 35, 0, true);
    }
    
    /*yellow*/
    steering(18, -30, 0, true);
    tank_turn(180, 0, 60);
    steering_time(1500, -50, 0);
    steering_time(500, -10, 0);
    steering(3, 30, 0, true);
    if(location[3] == CHEMICAL) {
        ev3_motor_rotate(EV3_PORT_A, 250, 30, true);
    }
    water(2);
    water(3);
    walltrace_length_p(13, 30, 10, 5);
    steering_color(COLOR_YELLOW, 35, 0);
    steering(7.9, 30, 0, true);
    map_check(4);
    chemical_taker(4);
    if(chemical_check == 2 && location[4] == CHEMICAL) {
        walltrace_length_p(7.5, 35, 9.5, 5);
    }
    else {
        steering(6.5, 30, 0, true);
        walltrace_length_p(16.5, 30, 9.5, 5);
    }
    tslp_tsk(1000*MSEC);
    /*red*/
    
    steering_color(COLOR_RED, 35, 0);
    steering(7.9, 30, 0, true);
    map_check(10);
    chemical_taker(10);
    if(chemical_check == 2 && location[10] == CHEMICAL) {
        steering(14.5, -35, 0, true);
    }

    steering(6, 35, 0, true);
    walltrace_length_p(9, 35, 9.5, 5);
    steering_time(700, 40, 0);
    map_check(11);
    if (location[11] == CHEMICAL) {
        steering(4, -30, 0, true);
        ev3_motor_rotate(EV3_PORT_A, 250, 15, true);
        steering(4, 30, 0, true);
    }

    if(location[10] == 3 || location[11] == 3) {
        steering(10, -35, 0, true);
        sp(10);
        sp(11);
        tslp_tsk(300 * MSEC);
        steering(9.5, 35, 0, true);
    }

    steering(9, -30, 0, true);
    tank_turn(178, -35, 35);
    water(11);
    water(10);
    steering(30.6, 35, 0, true);

    /*yellow*/

    tank_turn(90, 35, -35);
    steering_time(900, -45, 0);
    steering_time(500, -10, 0);
    tslp_tsk(200 * MSEC);
    tank_turn(117, 0, 40);
    tank_turn(116, 40, 0);
    steering(7.5, 27, 0, true);
    tslp_tsk(300 * MSEC);
    map_check(7);
    chemical_taker(7);
    if(location[7] == 3 || location[4] == 3) {
        steering(10, -35, 0, true);
        sp(7);
        sp(3);
        tslp_tsk(300 * MSEC);
        steering(9.5, 35, 0, true);
    }
    if(chemical_check == 2 && location[7] == CHEMICAL) {
        steering(3, -25, 0, true);
    }
    else {
        steering_color(COLOR_WHITE, 30, 0);
        steering(8, 27, 0, true);
    }
    water(4);
    water(7);
    /*white*/
    map_check(6);
    tslp_tsk(100*MSEC);
    chemical_taker(6);
    if(chemical_check == 2 && location[6] == CHEMICAL) {
        steering(22.5, 35, 0, true);
    }
    else {
        tslp_tsk(100*MSEC);
        steering_color(COLOR_WHITE, 30, 0);
        steering(36.9, 35, 0, true);
    }
    map_check(5);
    tslp_tsk(100*MSEC);
    chemical_taker(5);
    if(location[6] == 3 || location[5] == 3) {
        steering(10, -35, 0, true);
        sp(6);
        sp(5);
        tslp_tsk(300 * MSEC);
        steering(9.5, 35, 0, true);
    }

    water(5);
    water(6);

    brown = how_many - (location[0] + location[1] + location[2] + location[3] + location[4] + location[5] + location[6] + location[7] + location[10] + location[11]);

    int music_brown;
    music_brown = brown;
    while (music_brown == 0) {
        ev3_speaker_play_tone(NOTE_C5, 100);
        tslp_tsk(120*1000);
        music_brown = music_brown - 1;
    }

    if (brown == 0) {
        location[8] = NOTHING;
        location[9] = NOTHING;
        steering_color_b(COLOR_BLACK, 50, 0);
        steering(28, 35, 0, true);
        tank_turn(90, -35, 35);
        steering(12, -35, 0, true);
        if (location[3] == CHEMICAL) {
            steering(20, 30, 0, true);
            ev3_motor_rotate(EV3_PORT_A, 130, -40, true);
            tslp_tsk(400*MSEC);
            ev3_motor_set_power(EV3_PORT_A, -20);
            tslp_tsk(1300*MSEC);
            ev3_motor_stop(EV3_PORT_A, true);
            if (chemical_check == 2) {
                
            }
            steering_time(2200, -40, 0);
        }
        else {
            ev3_motor_set_power(EV3_PORT_A, -40);
            tslp_tsk(250*MSEC);
            ev3_motor_set_power(EV3_PORT_A, -20);
            tslp_tsk(1300*MSEC);
            ev3_motor_stop(EV3_PORT_A, true);
            if (chemical_check == 2) {
                
            }
            steering_time(1800, -40, 0);
        }
        tank_turn(180, 35, 0);
    }
    else if (brown == 5) {
        location[8] = FIRE;
        location[9] = NOTHING;
        tank_turn(180, -30, 0);
        steering(10, -30, 0, true);
        water(8);
        steering(10, 28, 0, true);
        tank_turn(180, 30, 0);
        steering_color_b(COLOR_BLACK, 50, 0);
        steering(29, 35, 0, true);
        tank_turn(90, -35, 35);
        steering(12, -35, 0, true);
        if (location[3] == CHEMICAL) {
            steering(20, 30, 0, true);
            ev3_motor_rotate(EV3_PORT_A, 130, -40, true);
            tslp_tsk(400*MSEC);
            ev3_motor_set_power(EV3_PORT_A, -20);
            tslp_tsk(1300*MSEC);
            ev3_motor_stop(EV3_PORT_A, true);
            if (chemical_check == 2) {
                
            }
            steering_time(2200, -40, 0);
        }
        else {
            ev3_motor_set_power(EV3_PORT_A, -40);
            tslp_tsk(250*MSEC);
            ev3_motor_set_power(EV3_PORT_A, -20);
            tslp_tsk(1300*MSEC);
            ev3_motor_stop(EV3_PORT_A, true);
            if (chemical_check == 2) {
                
            }
            steering_time(1800, -40, 0);
        }
        tank_turn(180, 35, 0);
    }
    else if (brown == 3) {
        location[8] = CHILD;
        location[9] = NOTHING;
        tank_turn(180, -30, 0);
        steering(28, -30, 0, true);
        sp(8);
        steering(27.5, 28, 0, true);
        tank_turn(180, 30, 0);
        steering_color_b(COLOR_BLACK, 50, 0);
        steering(29, 35, 0, true);
        tank_turn(90, -35, 35);
        steering(10, -35, 0, true);
        if (location[3] == CHEMICAL) {
            steering(20, 30, 0, true);
            ev3_motor_rotate(EV3_PORT_A, 130, -40, true);
            tslp_tsk(400*MSEC);
            ev3_motor_set_power(EV3_PORT_A, -20);
            tslp_tsk(1300*MSEC);
            ev3_motor_stop(EV3_PORT_A, true);
            if (chemical_check == 2) {
                
            }
            steering_time(2500, -40, 0);
        }
        else {
            ev3_motor_set_power(EV3_PORT_A, -40);
            tslp_tsk(250*MSEC);
            ev3_motor_set_power(EV3_PORT_A, -20);
            tslp_tsk(1300*MSEC);
            ev3_motor_stop(EV3_PORT_A, true);
            if (chemical_check == 2) {
                
            }
            steering_time(2000, -40, 0);
        }
        tank_turn(180, 35, 0);
    }
    else {
        steering_color_b(COLOR_BLACK, 50, 0);
        steering(11.5, 50, 0, false);
        tank_turn(75, 40, -40);
        tank_turn_color(25, -25);
        linetrace_length(11, 35);
        tslp_tsk(100*MSEC);
        linetrace_color(RIGHT, COLOR_BLACK, 30);
        ev3_speaker_play_tone(NOTE_A5, 100);
        steering(11, 30, 0, false);
        tank_turn(75, 40, -40);
        tank_turn_color(25, -25);
        linetrace_reflect(BOTH, 25, 20);
        /*brown*/
        tank_turn(46, 0, 30);
        tslp_tsk(100*1000);
        tank_turn(46, 30, 0);
        tslp_tsk(100*1000);
        map_check(8);
        if(chemical_check == 2 && location[8] == CHEMICAL) {

            tslp_tsk(500*1000);
            steering(10, 30, 0, true);
            ev3_motor_rotate(EV3_PORT_A, 200, -30, true);
            ev3_motor_rotate(EV3_PORT_A, 25, 30, true);
            steering(5, 30, 0, true);
            ev3_motor_rotate(EV3_PORT_A, 175, 15, true);
            steering(7, 25, 0, true);
            steering(14, 35, 0, true);
        }
        else {

            tslp_tsk(500*1000);
            if (location[8] == CHEMICAL) {
                steering(3, -28, 0, true);
                ev3_motor_rotate(EV3_PORT_A, 250, 15, true);
                steering(3, 28, 0, true);
            }
            tslp_tsk(500*1000);
            steering(7, 25, 0, true);
            steering(29, 30, 0, true);
        }
        
        tslp_tsk(300*MSEC);
        map_check(9);
        chemical_taker(9);
        if(location[8] == 3 || location[9] == 3) {
            steering(10, -35, 0, true);
            sp(8);
            sp(9);
            tslp_tsk(300 * MSEC);
            steering(10, 35, 0, true);
        }
        if(chemical_check == 2 && location[9] == CHEMICAL) {
            steering(14.5, -35, 0, true);
        }

        else {
            
        }
    

        /*location[11] = 16 - (location[0] + location[1] + location[2] + location[3] + location[4] + location[5] + location[6] + location[7] + location[8] + location[9] + location[10]);*/

        /*chemical*/
        /*if (location[11] == FIRE || location[10] == FIRE) {
            tank_turn(60, 0, 40);
            tank_turn(60, 40, 0);
            steering(12, 30, 0, true);
            tank_turn(160, -40, 40);
            tank_turn(35, 0, 30);
            water(11);
            water(10);
            steering(35, 30, 0, true);
        }*/
        
        steering(14, -30, 0, true);
        tank_turn(160, -40, 40);
        tank_turn(35, 0, 30);

        

        water(8);
        water(9);


        
        

        


        steering(9, 40, 0, false);
        walltrace_length_p(20, 60, 6, 5);
        steering_color(COLOR_WHITE, 35, 0);
        steering_color(COLOR_BLACK, 35, 0);
        steering(15, 40, 0, true);
        tank_turn(180, 0, 60);
        steering_time(1300, -30, 0);
        tslp_tsk(100*MSEC);
        steering(30, 70, 0, true);
        steering_color(COLOR_WHITE, 35, 0);
        steering_color(COLOR_BLACK, 35, 0);
        if (location[3] == CHEMICAL) {
            steering(20, 30, 0, true);
            ev3_motor_rotate(EV3_PORT_A, 130, -40, true);
            tslp_tsk(400*MSEC);
            ev3_motor_set_power(EV3_PORT_A, -20);
            tslp_tsk(1300*MSEC);
            ev3_motor_stop(EV3_PORT_A, true);
            if (chemical_check == 2) {
                
            }
            steering(48, -40, 0, true);
        }
        else {
            ev3_motor_set_power(EV3_PORT_A, -40);
            tslp_tsk(250*MSEC);
            ev3_motor_set_power(EV3_PORT_A, -20);
            tslp_tsk(1300*MSEC);
            ev3_motor_stop(EV3_PORT_A, true);
            if (chemical_check == 2) {
                
            }
            steering(28, -40, 0, true);
        }
        tank_turn(90, 40, -40);
    }
    
    if (location[0] == CHILD || location[0] == ADULT || location[1] == CHILD || location[1] == ADULT) map[0] = 1;
    if (location[2] == CHILD || location[2] == ADULT || location[3] == CHILD || location[3] == ADULT) map[1] = 1;
    if (location[5] == CHILD || location[5] == ADULT || location[6] == CHILD || location[6] == ADULT) map[2] = 1;
    if (location[4] == CHILD || location[4] == ADULT || location[7] == CHILD || location[7] == ADULT) map[3] = 1;
    if (location[8] == CHILD || location[8] == ADULT || location[9] == CHILD || location[9] == ADULT) map[4] = 1;
    if (location[10] == CHILD || location[10] == ADULT || location[11] == CHILD || location[11] == ADULT) map[5] = 1;

    arm(up);
    tslp_tsk(600*MSEC);
    walltrace_length(77, 67, 7);
    steering_time(1100, 30, 0);
    steering(11.5, -40, 0, true);
    tank_turn(90, -40, 40);
    steering_time(1500, -30, 0);
    /*walltrace_length_p(25, 25, 13, 20);
    walltrace_length_p(30, 30, 13, 5);*/
    walltrace_length(55, 40, 13);
    walltrace_length(10, 25, 13);
    ev3_motor_set_power(EV3_PORT_A, -30);
    tslp_tsk(1100*MSEC);
    ev3_motor_stop(EV3_PORT_A, true);
    tslp_tsk(100*MSEC);
    tank_turn(130, 0, -40);
    tank_turn(230, 40, 0);


    if (map[4] == 1) {
        steering(5, 30, 0, true);
        steering(3, -25, 0, true);
        ev3_motor_rotate(EV3_PORT_A, 120, 40, true);
        tank_turn(60, 0, 40);
        tank_turn(110, 0, -40);
        ev3_motor_set_power(EV3_PORT_A, -35);
        tslp_tsk(600*MSEC);
        ev3_motor_stop(EV3_PORT_A, true);
        tslp_tsk(100*MSEC);
        tank_turn(90, 0, 40);
        tank_turn(40, 0, -30);
        
        steering(15, -40, 0, true);
    }
    else {
        steering(13, -40, 0, true);
    }

    if (map[2] == 1) {
        ev3_motor_rotate(EV3_PORT_A, 120, 40, true);
        if (map[4] == 1) {
            steering(13, -40, 0, true);
            ev3_motor_set_power(EV3_PORT_A, -35);
            tslp_tsk(600*MSEC);
            ev3_motor_stop(EV3_PORT_A, true);
        }
        else {  
            tank_turn(60, 0, 40);
            tank_turn(110, 0, -40);
            ev3_motor_set_power(EV3_PORT_A, -35);
            tslp_tsk(600*MSEC);
            ev3_motor_stop(EV3_PORT_A, true);
            tslp_tsk(100*MSEC);
            tank_turn(90, 0, 40);
            tank_turn(40, 0, -30);
            
            steering(13, -40, 0, true);
        }
    }
    else {
        steering(13, -40, 0, true);
    }

    if (map[0] == 1) {
        ev3_motor_rotate(EV3_PORT_A, 120, 40, true);
        if (map[2] == 1 || map[4] == 1) {
            steering(13, -40, 0, true);
            ev3_motor_set_power(EV3_PORT_A, -35);
            tslp_tsk(600*MSEC);
            ev3_motor_stop(EV3_PORT_A, true);
        }
        else {
            tank_turn(60, 0, 40);
            tank_turn(110, 0, -40);
            ev3_motor_set_power(EV3_PORT_A, -35);
            tslp_tsk(600*MSEC);
            ev3_motor_stop(EV3_PORT_A, true);
            tslp_tsk(100*MSEC);
            tank_turn(90, 0, 40);
            tank_turn(40, 0, -30);
            steering(13, -40, 0, true);
        }
    }
    else {
        steering(8, -40, 0, true);
    }

    steering_time(1500, -40, 0);
    
    switch (start) {
        case 2:
            if (map[0] + map[2] + map[4] == 2) {
                steering(19, 40, 0, true);
                tank_turn(95, 40, -40);
                tank_turn(170, 40, 0);
            }
            else {
                steering(15, 40, 0, true);
                tank_turn(100, 40, -40);
                tank_turn(160, 40, 0);
            }
            steering_time(650, 35, 0);
            steering(45, -40, 0, true);


            if (map[1] == 1) {
                steering(10, -40, 0, true);
                tank_turn(50, 30, 0);
                steering(6, 30, 0, true);
                steering(4, -24, 0, true);
                ev3_motor_rotate(EV3_PORT_A, 115, 40, true);
                steering(6, -30, 0, true);
                tank_turn(50, 0, 30);
                steering(5, 30, 0, true);
                ev3_motor_set_power(EV3_PORT_A, -35);
                tslp_tsk(600*MSEC);
                ev3_motor_stop(EV3_PORT_A, true);
                
                
                steering(14, -40, 0, true);
                
                
            
            }
            else {
                steering(14, -40, 0, true);
            }

            if (map[3] == 1) {
                steering(10, -40, 0, true);
                tank_turn(50, 30, 0);
                steering(6, 30, 0, true);
                steering(4, -24, 0, true);
                ev3_motor_rotate(EV3_PORT_A, 115, 40, true);
                steering(6, -30, 0, true);
                tank_turn(50, 0, 30);
                steering(5, 30, 0, true);
                ev3_motor_set_power(EV3_PORT_A, -35);
                tslp_tsk(600*MSEC);
                ev3_motor_stop(EV3_PORT_A, true);
                
                
                steering(14, -40, 0, true);
            }
            else {
                steering(14, -40, 0, true);
            }

            if (map[5] == 1) {
                steering(10, -40, 0, true);
                tank_turn(50, 30, 0);
                steering(6, 30, 0, true);
                steering(4, -24, 0, true);
                ev3_motor_rotate(EV3_PORT_A, 115, 40, true);
                steering(6, -30, 0, true);
                tank_turn(50, 0, 30);
                steering(5, 30, 0, true);
                ev3_motor_set_power(EV3_PORT_A, -35);
                tslp_tsk(600*MSEC);
                ev3_motor_stop(EV3_PORT_A, true);
                
                steering(13.5, -40, 0, true);
            }
            else {
                steering(11, -40, 0, true);
            }
            
            


            /*goal*/

            steering(4, -27, 0, true);

            steering_time(1000, -30, 0);
            steering(2.7, 30, 0, true);
            tank_turn(90, -35, 35);

            ev3_motor_stop(EV3_PORT_A, true);
            if (map[0] + map[2] + map[4] == 2) {
                steering_time(1500, -40, -10);
            }
            else {
                steering_time(1200, -40, -10);
            }


            
            break;
    

        case 1:
            tank_turn(140, 40, 0);
            tank_turn(140, 0, 40);
            steering(33, 35, 0, true);
          
            
            if (map[5] == 1) {
                steering(10, -40, 0, true);
                tank_turn(50, 30, 0);
                steering(6, 30, 0, true);
                steering(4, -22, 0, true);
                ev3_motor_rotate(EV3_PORT_A, 115, 40, true);
                steering(6, -30, 0, true);
                tank_turn(50, 0, 30);
                steering(5, 30, 0, true);
                ev3_motor_set_power(EV3_PORT_A, -35);
                tslp_tsk(600*MSEC);
                ev3_motor_stop(EV3_PORT_A, true);
                steering(14, -40, 0, true);
            }
            else {
                steering(16.5, -30, 0, true);
            }

            if (map[3] == 1) {
                steering(10, -40, 0, true);
                tank_turn(50, 30, 0);
                steering(6, 30, 0, true);
                steering(4, -22, 0, true);
                ev3_motor_rotate(EV3_PORT_A, 115, 40, true);
                steering(6, -30, 0, true);
                tank_turn(50, 0, 30);
                steering(5, 30, 0, true);
                ev3_motor_set_power(EV3_PORT_A, -35);
                tslp_tsk(600*MSEC);
                ev3_motor_stop(EV3_PORT_A, true);
                steering(14, -40, 0, true);
            }
            else {
                steering(16.5, -30, 0, true);
            }

            if (map[1] == 1) {
                ev3_motor_rotate(EV3_PORT_A, 115, 40, true);

                steering(10, -30, 0, true);
            }
            else {
                steering(10, -30, 0, true);
            }
            steering_time(1000, -30, 0);
            steering(12.2, 30, 0, true);
            tank_turn(180, 0, -40);
            ev3_motor_set_power(EV3_PORT_A, -35);
            tslp_tsk(600*MSEC);
            ev3_motor_stop(EV3_PORT_A, true);
            steering_time(2000, -25, 0); 
            
            ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_C, true);

            break;
    }

    ev3_speaker_play_tone(NOTE_A6, 100);
    sensor_check(location_sensor[0]);
    tslp_tsk(200*MSEC);
    ev3_speaker_play_tone(NOTE_A6, 100);
    sensor_check(location_sensor[1]);
    tslp_tsk(200*MSEC);
    ev3_speaker_play_tone(NOTE_A6, 100);
    sensor_check(location_sensor[2]);
    tslp_tsk(200*MSEC);
    ev3_speaker_play_tone(NOTE_A6, 100);
    sensor_check(location_sensor[3]);
    tslp_tsk(200*MSEC);
    ev3_speaker_play_tone(NOTE_A6, 100);
    sensor_check(location_sensor[4]);
    tslp_tsk(200*MSEC);
    ev3_speaker_play_tone(NOTE_A6, 100);
    sensor_check(location_sensor[5]);
    tslp_tsk(200*MSEC);
    ev3_speaker_play_tone(NOTE_A6, 100);
    sensor_check(location_sensor[6]);
    tslp_tsk(200*MSEC);
    ev3_speaker_play_tone(NOTE_A6, 100);
    sensor_check(location_sensor[7]);
    tslp_tsk(200*MSEC);
    ev3_speaker_play_tone(NOTE_A6, 100);
    sensor_check(location_sensor[8]);
    tslp_tsk(200*MSEC);
    ev3_speaker_play_tone(NOTE_A6, 100);
    sensor_check(location_sensor[9]);
    tslp_tsk(200*MSEC);
    ev3_speaker_play_tone(NOTE_A6, 100);
    sensor_check(location_sensor[10]);
    tslp_tsk(200*MSEC);
    ev3_speaker_play_tone(NOTE_A6, 100);
    sensor_check(location_sensor[11]);
    
    while(1) {}

}