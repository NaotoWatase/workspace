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
#include "math.h"


#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

#define MSEC (1000)
#define ROBOT1CM (18.48)
#define TURN (0.163)

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
            if(length*ROBOT1CM <= motor_count && length*ROBOT1CM > 0) break;
            if(length*ROBOT1CM >= motor_count && length*ROBOT1CM < 0) break;
        }
    }
    else {
        (void)ev3_motor_set_power(EV3_PORT_B, -(power+(power*steering/50)));
        (void)ev3_motor_set_power(EV3_PORT_C, power);
        while (true) {
            motor_count = ev3_motor_get_counts(EV3_PORT_C);
            if(length*ROBOT1CM <= motor_count && length*ROBOT1CM > 0) break;
            if(length*ROBOT1CM >= motor_count && length*ROBOT1CM < 0) break;
        }
    }

    if(brake == true) {
        ev3_motor_stop(EV3_PORT_B, true);
        ev3_motor_stop(EV3_PORT_C, true);
    }
}

void tank_turn(float angle, int power_L, int power_R, bool_t brake){
    int last_count;
    int motor_count;
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    if (power_R == 0) {
        (void)ev3_motor_set_power(EV3_PORT_B, (int16_t)-power_L);
    } 
    if (power_L == 0) {
        (void)ev3_motor_set_power(EV3_PORT_C, (int16_t)power_R);
    }
    if (power_L != 0 && power_R != 0) {
        (void)ev3_motor_set_power(EV3_PORT_B, (int16_t)-power_L);
        (void)ev3_motor_set_power(EV3_PORT_C, (int16_t)power_R);

    }
    while (true) {
        if (power_R == 0) {
            motor_count = ev3_motor_get_counts(EV3_PORT_B);
        }
        if (power_L == 0) {
            motor_count = ev3_motor_get_counts(EV3_PORT_C);
        }
        if (power_L != 0 && power_R != 0) {
            motor_count = ev3_motor_get_counts(EV3_PORT_C);
        }
        last_count = abs(motor_count);
        if(angle*TURN*ROBOT1CM <= last_count) break;
    }
}

void tank_turn_color(int power_L, int power_R, bool_t brake){

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

void steering_color(colorid_t color_stop, int power, int steering, bool_t brake){
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

void steering_color_u(colorid_t color_stop, int power, int steering, bool_t brake){
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

void steering_color_b(colorid_t color_stop, int power, int steering, bool_t brake){
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

void steering_time(int time_stop_4d, int power, int steering, bool_t brake){
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

void linetrace_color(sensortype_t type, colorid_t color_stop, int power, bool_t brake){
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
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
    ev3_speaker_play_tone(NOTE_AS5, 100);
    tslp_tsk(100*MSEC);
    
}

void linetrace_reflect(sensortype_t type, int reflect_stop, int power, bool_t brake){

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
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
    ev3_speaker_play_tone(NOTE_AS5, 100);
    tslp_tsk(100*MSEC);
    
}


void walltrace_length(float length, int power, float dist, bool_t brake){
    int PGEIN = 20;
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

void walltrace_length_p(float length, int power, float dist, int pgein, bool_t brake){
    int PGEIN = pgein;
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

void linetrace_length(float length, int power, bool_t brake){
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
    while (ct < num) {
        ev3_speaker_play_tone(NOTE_C5, 100);
        tslp_tsk(120*1000);
        ct = ct + 1;
    }
}

void chemical_taker(int n){
    armc = 0;
    if(location[n] == CHEMICAL){
        steering(14.5, 30, 0, true);
        ev3_motor_rotate(EV3_PORT_A, 270, 30, true);
        tslp_tsk(750*1000);
    }

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
        location_sensor[num] = obj;
        switch (obj){
            case 1:
            case 2:
            case 3:
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
            case 11:
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
                break;
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
        ev3_motor_rotate(EV3_PORT_A, 270, -30, true);
        tslp_tsk(400*MSEC);
        break;
    case up:
        ev3_motor_rotate(EV3_PORT_A, 270, 30, true);
        tslp_tsk(400*MSEC);
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

    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);
    steering(3, 100, 0, true);


    tslp_tsk(10000* MSEC);

    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, false);
    steering(3, 100, 0, true);


    
    

    while(1) {}





}
