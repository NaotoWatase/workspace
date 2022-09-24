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
#define TURN (0.162)

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
static const motor_port_t   PortMotorArm1  = EV3_PORT_A;
static const motor_port_t   PortMotorArm2  = EV3_PORT_D;
static const sensor_port_t PortSensorGYRO = EV3_PORT_4;
static const sensor_port_t PortSensorColor1 = EV3_PORT_1;
static const sensor_port_t PortSensorColor2 = EV3_PORT_2;
static const sensor_port_t PortSensorColor3 = EV3_PORT_3;



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
int chemical_check;
int judgement_check;
int chemical;


int white;
int how_many = 31;

int map [6] = {0,0,0,0,0,0};
int start;

int chemical_type;

int water_count;




/*mapのデータ変数*/
typedef enum object {  
    PERSON = 10,
    CHILD = 3,
    ADULT = 2,
    FIRE = 5,
    CHEMICAL = 1,
    NOTHING = 0
} object_t ;

/*mapceck時のセンサーを指定*/
typedef enum way{
    RIGHT,
    LEFT
} way_t ;



typedef enum UpDown {
    up = 1,
    down = 0
} arm_t ;

uint8_t obj = 0;
uint8_t test = 0;

int location[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

int location_sensor[12] = {0,0,0,0,0,0,0,0,0,0,0,0};


rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)

int red=0;
int green=0;
int blue=0;
int judgement;


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


void steering_time(int time_stop_4d, int power, int steering){
    power = -power;
    if(steering > 0) {
    (void)ev3_motor_set_power(EV3_PORT_B, power);
    (void)ev3_motor_set_power(EV3_PORT_C, power+(power*steering/50));
    }
    else {
    (void)ev3_motor_set_power(EV3_PORT_B, power-(power*steering/50));
    (void)ev3_motor_set_power(EV3_PORT_C, power);
    }
    tslp_tsk(time_stop_4d * MSEC);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}


void steering_color(colorid_t color_stop, int power, int steering){
    colorid_t color;
    power = -power;
    if(steering > 0) {
    (void)ev3_motor_set_power(EV3_PORT_B, power);
    (void)ev3_motor_set_power(EV3_PORT_C, power+(power*steering/50));
    }
    else {
    (void)ev3_motor_set_power(EV3_PORT_B, power-(power*steering/50));
    (void)ev3_motor_set_power(EV3_PORT_C, power);
    }
    while (color_stop != color) {
        color = ev3_color_sensor_get_color(EV3_PORT_1);
    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);    
}



void p_turn(int angle, int left_motor, int right_motor){
    angle = angle * 181 / 180;
    int turn_check = 0;
    int gyro = 0;
    int power;
    ev3_gyro_sensor_reset(EV3_PORT_4);
    while (true) {
        gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        gyro = abs(gyro);
        power = (65 / (angle / 10 * 1)) * gyro;
        if(power < 20) power = 20;
        if(power > 65) power = 65;
        if (gyro > angle / 10 * 1) turn_check = turn_check + 1;

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
        if (turn_check > 0) break;
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
    if(angle - gyro == 0) {
        ev3_speaker_play_tone(NOTE_A4, 100);
        tslp_tsk(1000);
    }
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


void linetrace_length(float length, int power){
    ev3_motor_reset_counts(EV3_PORT_C);
    float p = 0;
    float i = 0;
    float d = 0;
    float d2 = 0;
    int reflect;
    power = -power;
    while (true) {
        kakudo_C = ev3_motor_get_counts(EV3_PORT_C);
        reflect = ev3_color_sensor_get_reflect(EV3_PORT_1);
        
        p = reflect - 35;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  -p * P_GEIN + i * 0 + d * 0; 
        if (length * ROBOT1CM < -kakudo_C) break;
        if(steer > 0) {
        (void)ev3_motor_set_power(EV3_PORT_B, power);
        (void)ev3_motor_set_power(EV3_PORT_C, power+(power*steer/50));
        }
        else {
        (void)ev3_motor_set_power(EV3_PORT_B, power-(power*steer/50));
        (void)ev3_motor_set_power(EV3_PORT_C, power);
        }

    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
}


void map_check(int num, way_t sensor) {
    /*ev3_speaker_play_tone(NOTE_A5, 100);
    tslp_tsk(300*MSEC);*/
    
    if(sensor == RIGHT){
        while ( !ht_nxt_color_sensor_measure_color(EV3_PORT_3, &test) ) {
            ;
        }

        while ( !ht_nxt_color_sensor_measure_color(EV3_PORT_3, &obj) ) {
            ;
        }      
        while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_3, &rgb_val)) {
            ;
        }  
    } 
    else{
        while ( !ht_nxt_color_sensor_measure_color(EV3_PORT_2, &test) ) {
            ;
        }

        while ( !ht_nxt_color_sensor_measure_color(EV3_PORT_2, &obj) ) {
            ;
        }      
        while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_2, &rgb_val)) {
            ;
        }  
    }
    
    red = rgb_val.r;
    green = rgb_val.g;
    blue = rgb_val.b;
    judgement = red + green + blue;
    
    location_sensor[num] = obj;

    switch (obj){
        case 1:
        case 2:
        case 3:
        case 4:
            location[num] = PERSON;
            break;
        case 12:
        case 13:
        case 14:
        case 17:
            judgement_check = judgement;
            if (judgement > 30) {
                location[num] = PERSON;
            } 
            else {
                location[num] = NOTHING;
            }
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
            chemical_check = chemical_check + 1;
            location[num] = CHEMICAL;
            break; 
        default:
            location[num] = NOTHING;
            break;
    } 
    /*ev3_speaker_play_tone(NOTE_A5, 100);
    tslp_tsk(400*MSEC);*/
}

void chemical_taker(int n, way_t sensor){
    if(location[n] == CHEMICAL){
        chemical = chemical + 1;
       if(sensor == RIGHT){
           ev3_motor_rotate(EV3_PORT_A, 270, -20, true);
           chemical_type = RIGHT;
        }
        else{
            ev3_motor_rotate(EV3_PORT_A, 270, 20, true);
            chemical_type = LEFT;
        }
    }
}

void water(int n) {
    if (location[n] == FIRE) {
        ev3_motor_rotate(EV3_PORT_D, 80 + water_count, 20, true);
        tslp_tsk(100*MSEC);
        ev3_motor_rotate(EV3_PORT_D, 80 + water_count, -20, false);
        water_count = water_count + 100;
    }
}


void sensor_check(uint8_t num) {
    int ct = 0;
    while (ct < num) {
        ev3_speaker_play_tone(NOTE_C5, 100);
        tslp_tsk(500*1000);
        ct = ct + 1;
    }
}



void main_task(intptr_t unused){

    
    /* Register button handlers */
    ev3_button_set_on_clicked(BACK_BUTTON, &button_clicked_handler, BACK_BUTTON);

    /* Configure motors */
    ev3_motor_config(PortMotorLeft, LARGE_MOTOR);
    ev3_motor_config(PortMotorRight, LARGE_MOTOR);
    ev3_motor_config(PortMotorArm1, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorArm2, MEDIUM_MOTOR);



    /* Configure sensors */

    ev3_sensor_config(PortSensorGYRO, GYRO_SENSOR);
    ev3_sensor_config(PortSensorColor1, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor2, HT_NXT_COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor3, HT_NXT_COLOR_SENSOR);


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
    start = 1;

    

    
   /* ev3_motor_rotate(EV3_PORT_A, 160,20, true);
    tslp_tsk(1000 * MSEC);
    ev3_motor_rotate(EV3_PORT_A, 160,-20, true);*/
    

}