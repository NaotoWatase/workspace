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
int steer;
int powers1;
int powers2;
int powers3;
int powers4;
int armc;
int chemical_check = 0;

int music;
int kaisuu;

float distance;


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

typedef enum UpDown {
    up = 1,
    down = 0
} arm_t ;

int location[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

uint8_t test = 0;
uint8_t obj = 0;

rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)

int red=0;
int green=0;
int blue=0;

int water_count = 0;

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
    ev3_speaker_play_tone(NOTE_AS5, 100);
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
    
}


void walltrace_length(float length, int power, float dist){
    int PGEIN = 20;
    ev3_motor_reset_counts(EV3_PORT_C);
    while (true) {
        kakudo_C = ev3_motor_get_counts(EV3_PORT_C);
        int dist1 = ev3_ultrasonic_sensor_get_distance(EV3_PORT_1);
        if (dist1 > 50) {
            dist1 = 49; 
        } 
        p = dist1 - (dist + 1);
        steer =  p * PGEIN;
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

void walltrace_length_p(float length, int power, float dist, int pgein){
    int PGEIN = pgein;
    ev3_motor_reset_counts(EV3_PORT_C);
    while (true) {
        kakudo_C = ev3_motor_get_counts(EV3_PORT_C);
        int dist1 = ev3_ultrasonic_sensor_get_distance(EV3_PORT_1);
        if (dist1 > 50) {
            dist1 = 49; 
        } 
        p = dist1 - (dist + 1);
        steer =  p * PGEIN;
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

void linetrace_length(float length, int power){
    ev3_motor_reset_counts(EV3_PORT_C);
    while (true) {
        kakudo_C = ev3_motor_get_counts(EV3_PORT_C);
        reflect2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        reflect3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        reflect = reflect2 - reflect3;
        p = reflect;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  p * P_GEIN; 
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
        ev3_motor_rotate(EV3_PORT_D, 25 + water_count, 20, true);
        ev3_motor_rotate(EV3_PORT_D, 25 + water_count, -20, false);
        water_count = water_count + 80;
    }
}

void sensor_check(uint8_t num) {
    int ct = 0;
    while (ct < num + 1) {
        ev3_speaker_play_tone(NOTE_C5, 100);
        tslp_tsk(120*1000);
        ct = ct + 1;
    }
}

void chemical_taker(int n){
    armc = 0;
    /*if(location[n] == CHEMICAL){
        ev3_motor_reset_counts(EV3_PORT_A);
        steering(3, -20, 0);
        ev3_motor_set_power(EV3_PORT_A, 25);
        while (true) {
            armc = ev3_motor_get_counts(EV3_PORT_A);
            if (armc > 189)break;
        }
        ev3_motor_stop(EV3_PORT_A, true);
        steering(3, 20, 0);

        if (armc < 189) {
            ev3_motor_set_power(EV3_PORT_A, 25);
            while (true) {
                armc = ev3_motor_get_counts(EV3_PORT_A);
                if (armc > 189)break;
            }
        }
        ev3_motor_stop(EV3_PORT_A, true);
    }*/
}

void map_check(int num) {

    tslp_tsk(200 * MSEC);

    while ( !ht_nxt_color_sensor_measure_color(EV3_PORT_4, &test) ) {
        ;
    }

    
    while ( !ht_nxt_color_sensor_measure_color(EV3_PORT_4, &obj) ) {
        ;
    }      

    distance = ev3_ultrasonic_sensor_get_distance(EV3_PORT_1);
    if (distance < 6) {
        switch (obj){
            case 1:
            case 2:
            case 3:
                location[num] = ADULT;
                sensor_check(obj);
                break;
            case 4:
            case 13:
                location[num] = CHILD;
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
                if (chemical_check > 0) {
                    switch (obj) {
                        default:
                            break;
                    }
                }
                else {
                    chemical_check = chemical_check + 1;
                    location[num] = CHEMICAL;
                    sensor_check(obj);
                    break;
                }
                
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
                sensor_check(obj);
                break;
            case 4:
                location[num] = CHILD;
                sensor_check(obj);
                break;
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
            case 10:
                location[num] = FIRE;
                sensor_check(obj);
                break;
            default:
                location[num] = NOTHING;
                break;
        }
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
        ev3_motor_rotate(EV3_PORT_A, 325, 30, true);
        tslp_tsk(400*MSEC);
        break;
    case up:
        ev3_motor_set_power(EV3_PORT_A, -50);
        tslp_tsk(900*MSEC);
        break;
    }
    ev3_motor_stop(EV3_PORT_A, true);
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

    

    /*スタートの分岐チェック*/
    start = 2;

    /*スタート*/
    switch(start){
        case 1:
            steering(65, 50, 0);
            steering_color(COLOR_WHITE, 30, 0);
            steering_time(400, 25, 0);
            tank_turn(180, 0, 40);
            walltrace_length(57, 40, 8.5);
            walltrace_length(14, 25, 8.5);
            steering_color(COLOR_WHITE, 30, 0);
            linetrace_length(25, 30);
            linetrace_color(BOTH, COLOR_BLUE, 20);
            break;
        case 2:
            walltrace_length(75, 40, 8.5);
            walltrace_length(14, 25, 8.5);
            steering_color(COLOR_WHITE, 30, 0);
            linetrace_length(25, 30);
            linetrace_color(BOTH, COLOR_BLUE, 20);
            break;
    }
    
    /*blue*/
    steering(6.5, 35, 0);
    map_check(0);
    chemical_taker(0);
    tank_turn(25, 0, 25);
    tank_turn(25, 25, 0);
    steering(4.5, 30, 0);
    walltrace_length_p(18.5, 32, 10, 20);
    steering(10, 35, 0);
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
    water(0);
    water(1);
    /*green*/
    steering(11, 30, 0);
    map_check(2);
    chemical_taker(2);
    steering(6.5, 30, 0);
    walltrace_length_p(20, 25, 10.1, 20);
    /*steering(35, 25, 0);*/
    steering_time(1000, 20, 0);
    map_check(3);
    /*yellow*/
    steering(16.5, -30, 0);
    tank_turn(180, 0, 30);
    steering_time(1500, -50, 0);
    steering_time(500, -10, 0);
    water(2);
    water(3);
    walltrace_length_p(13, 25, 9, 20);
    steering_color(COLOR_YELLOW, 35, 0);
    steering(7.9, 30, 0);
    map_check(4);
    chemical_taker(4);
    tslp_tsk(1000*MSEC);
    /*red*/
    steering(6.5, 30, 0);
    walltrace_length_p(16.5, 25, 9, 20);
    steering_color(COLOR_RED, 35, 0);
    steering(7.9, 30, 0);
    map_check(10);
    chemical_taker(10);

    /*yellow*/

    tank_turn(180, -35, 0);
    steering_time(500, 30, 0);
    steering_time(900, -50, 0);
    steering_time(500, -10, 0);
    tank_turn(104, 0, 25);
    tank_turn(104, 25, 0);
    steering(8.4, 27, 0);
    tslp_tsk(300 * MSEC);
    map_check(7);
    water(4);
    water(7);
    /*white*/
    steering_color(COLOR_WHITE, 30, 0);
    steering(8, 27, 0);
    map_check(6);
    chemical_taker(6);
    steering(36.9, 35, 0);
    map_check(5);
    chemical_taker(5);
    water(5);
    water(6);
    linetrace_color(BOTH, COLOR_BLACK, 30);
    tank_turn(140, 35, 0);
    tank_turn_color(25, 0);
    linetrace_length(29, 30);
    ev3_speaker_play_tone(NOTE_A5, 100);
    tank_turn(75, 30, -30);
    tank_turn_color(25, -25);
    linetrace_reflect(BOTH, 20, 22);
    /*brown*/
    steering(7, 25, 0);
    map_check(8);
    chemical_taker(8);
    tank_turn(50, 0, 20);
    tank_turn(50, 20, 0);
    steering(29, 30, 0);
    tslp_tsk(300*MSEC);
    map_check(9);
    chemical_taker(9);
  

    location[11] = 16 - (location[0] + location[1] + location[2] + location[3] + location[4] + location[5] + location[6] + location[7] + location[8] + location[9] + location[10]);

    /*chemical*/
    if (location[11] == FIRE || location[10] == FIRE) {
        tank_turn(60, 0, 25);
        tank_turn(60, 25, 0);
        steering(12, 30, 0);
        tank_turn(180, -25, 25);
        water(11);
        water(10);
        steering(35, 30, 0);
    }
    else {
        steering(14, -30, 0);
        tank_turn(180, -25, 25);

    }

    water(8);
    water(9);

    /*int n = 0;

    while (1) {
        switch (location[n]) {
                    
            case ADULT:
                ev3_speaker_play_tone(NOTE_C5, 100);
                tslp_tsk(100*1000);
                ev3_speaker_play_tone(NOTE_C5, 100);
                tslp_tsk(100*1000);
                ev3_speaker_play_tone(NOTE_C5, 100);
                tslp_tsk(100*1000);
                ev3_speaker_play_tone(NOTE_C5, 100);
                break;

            case CHILD:
                ev3_speaker_play_tone(NOTE_C5, 100);
                tslp_tsk(100*1000);
                ev3_speaker_play_tone(NOTE_C5, 100);
                tslp_tsk(100*1000);
                ev3_speaker_play_tone(NOTE_C5, 100);
                break;

            case FIRE:
                ev3_speaker_play_tone(NOTE_C5, 100);
                tslp_tsk(100*1000);
                ev3_speaker_play_tone(NOTE_C5, 100);
                tslp_tsk(100*1000);
                ev3_speaker_play_tone(NOTE_C5, 100);
                tslp_tsk(100*1000);
                ev3_speaker_play_tone(NOTE_C5, 100);
                tslp_tsk(100*1000);
                ev3_speaker_play_tone(NOTE_C5, 100);
                break;

            case CHEMICAL:
                ev3_speaker_play_tone(NOTE_C5, 100);
                tslp_tsk(100*1000);
                ev3_speaker_play_tone(NOTE_C5, 100);
                break;

            case NOTHING:
                ev3_speaker_play_tone(NOTE_C5, 100);
                break;

            default:
                ev3_speaker_play_tone(NOTE_C6, 100);
                break;
        }
        tslp_tsk(500*1000);
        n = n + 1;
        if (n == 12) {
            break;
        }    
    } */
    
    int map [6] = {0,0,0,0,0,0};

    if (location[0] == CHILD || location[0] == ADULT || location[1] == CHILD || location[1] == ADULT) map[0] = 1;
    if (location[2] == CHILD || location[2] == ADULT || location[3] == CHILD || location[3] == ADULT) map[1] = 1;
    if (location[5] == CHILD || location[5] == ADULT || location[6] == CHILD || location[6] == ADULT) map[2] = 1;
    if (location[4] == CHILD || location[4] == ADULT || location[7] == CHILD || location[7] == ADULT) map[3] = 1;
    if (location[8] == CHILD || location[8] == ADULT || location[9] == CHILD || location[9] == ADULT) map[4] = 1;
    if (location[10] == CHILD || location[10] == ADULT || location[11] == CHILD || location[11] == ADULT) map[5] = 1;


    walltrace_length(41, 50, 7);

    ev3_motor_rotate(EV3_PORT_A, 80, 50, true);

    arm(up);
    tslp_tsk(600*MSEC);
    walltrace_length(110, 60, 7);
    steering_time(1100, 30, 0);
    steering(11.5, -25, 0);
    tank_turn(90, -30, 30);
    steering_time(1500, -30, 0);
    walltrace_length(55, 30, 13);
    steering(10, 30, 0);
    arm(down);
    tank_turn(130, 0, -30);
    tank_turn(230, 30, 0);


    if (map[4] == 1) {
        ev3_motor_rotate(EV3_PORT_A, 70, -70, true);
        tank_turn(60, 0, 25);
        tank_turn(100, 0, -25);
        ev3_motor_rotate(EV3_PORT_A, 70, 70, true);
        tank_turn(40, 0, 25);
        
        steering(13, -25, 0);
    }
    else {
        steering(13, -30, 0);
    }

    if (map[2] == 1) {
        ev3_motor_rotate(EV3_PORT_A, 70, -70, true);
        tank_turn(60, 0, 25);
        tank_turn(100, 0, -25);
        ev3_motor_rotate(EV3_PORT_A, 70, 70, true);
        tank_turn(40, 0, 25);
        
        steering(13, -25, 0);
    }
    else {
        steering(13, -25, 0);
    }

    if (map[0] == 1) {
        ev3_motor_rotate(EV3_PORT_A, 70, -70, true);
        tank_turn(60, 0, 25);
        tank_turn(100, 0, -25);
        ev3_motor_rotate(EV3_PORT_A, 70, 70, true);
        tank_turn(40, 0, 25);
        
        steering(13, -25, 0);
    }
    else {
        steering(8, -25, 0);
    }

    steering_time(1800, -25, 0);
    
    switch (start) {
        case 2:
            steering(15, 25, 0);
            tank_turn(110, 25, -25);
            tank_turn(140, 25, 0);
            steering(42, -30, 0);


            if (map[1] == 1) {
                steering(10, -25, 0);
                tank_turn(50, 25, 0);
                steering(6, 30, 0);
                steering(4, -25, 0);
                ev3_motor_rotate(EV3_PORT_A, 70, -70, true);
                steering(6, -25, 0);
                tank_turn(50, 0, 25);
                steering(5, 25, 0);
                ev3_motor_rotate(EV3_PORT_A, 70, 70, true);
                
                
                steering(14, -25, 0);
                
                
            
            }
            else {
                steering(14.5, -20, 0);
            }

            if (map[3] == 1) {
                steering(10, -25, 0);
                tank_turn(50, 25, 0);
                steering(6, 30, 0);
                steering(4, -25, 0);
                ev3_motor_rotate(EV3_PORT_A, 70, -70, true);
                steering(6, -25, 0);
                tank_turn(50, 0, 25);
                steering(5, 25, 0);
                ev3_motor_rotate(EV3_PORT_A, 70, 70, true);
                
                
                steering(14, -25, 0);
            }
            else {
                steering(14.5, -20, 0);
            }

            if (map[5] == 1) {
                steering(10, -25, 0);
                tank_turn(50, 25, 0);
                steering(6, 30, 0);
                steering(4, -25, 0);
                ev3_motor_rotate(EV3_PORT_A, 70, -70, true);
                steering(6, -25, 0);
                tank_turn(50, 0, 25);
                steering(5, 25, 0);
                ev3_motor_rotate(EV3_PORT_A, 70, 70, true);
                
                
                steering(13.5, -25, 0);
            }
            else {
                steering(11, -20, 0);
            }
            
            arm(up);
            steering(4, -27, 0);
            tank_turn(90, -27, 27);
            ev3_motor_stop(EV3_PORT_A, true);
            steering_time(2000, -25, -10);
            
            break;
    
        case 1:
            tank_turn(160, 30, 0);
            tank_turn(160, 0, 30);
            steering(35, 35, 0);


            if (map[5] == 1) {
                ev3_motor_rotate(EV3_PORT_A, 70, -70, true);
                steering(3, -20, 0);
                tank_turn(45, 0, 25);
                tank_turn(45, 0, -25);
                steering(5, 25, 0);
                ev3_motor_rotate(EV3_PORT_A, 70, 70, true);
                steering(17, -25, 0);
            }
            else {
                steering(15, -25, 0);
            }

            if (map[3] == 1) {
                ev3_motor_rotate(EV3_PORT_A, 70, -70, true);
                steering(3, -20, 0);
                tank_turn(45, 0, 25);
                tank_turn(45, 0, -25);
                steering(5, 25, 0);
                ev3_motor_rotate(EV3_PORT_A, 70, 70, true);
                steering(17, -25, 0);
            }
            else {
                steering(15, -25, 0);
            }

            if (map[1] == 1) {
                ev3_motor_rotate(EV3_PORT_A, 70, -70, true);

                steering(11, -25, 0);
            }
            else {
                steering(11, -25, 0);
            }
            tank_turn(180, 0, -27);
            arm(up);
            steering_time(2000, -25, 0);
            
            ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_C, true);

            break;
    }
    
    while(1) {}





}
