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
#include <stdio.h>
#include "math.h"
#include "wave.h"

/* /dev/tty.MindstormsEV3-SerialPor */

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

static FILE *bt = NULL;

#define MSEC (1000)
#define ROBOT1CM (18.48)
#define TURN (0.162)

void newsteering(int power, float cm);

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


static int32_t fontw, fonth;

int power = 50;

float distance = 0;
int judgement_check = 0;
int chemical = 0;


int white = 0;
int how_many = 11;//31

int map [6] = {0,0,0,0,0,0};
int start = 2;

int chemical_type = 0;

int water_count = 0;

int y = 0;

bool_t isArmUp = false;



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
    LEFT,
    NONE
} way_t ;

uint8_t obj = 0;
uint8_t test = 0;
rgb_raw_t testrgb;


int location[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0};

/*マルチタスク lovation_task用*/
int savedate_color[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0};
int savedate_red[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0};
int savedate_green[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0};
int savedate_blue[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0};
int location_counts = 0;
int task_counts = 0;

int location_sensor[12] = {0,0,0,0,0,0,0,0,0,0,0,0};


rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)

int red=0;
int green=0;
int blue=0;
int judgement = 0;


void beep() {
    ev3_speaker_play_tone(NOTE_C5, 10);
}

int volt() {
    return ev3_battery_voltage_mV();
}

void print_str(char* str, int line) {
    ev3_lcd_fill_rect(0, fonth * line, EV3_LCD_WIDTH, fonth, EV3_LCD_WHITE);
    ev3_lcd_draw_string(str, 0, fonth * line);                     // line 行目に出力
}

void print_map() {
    char buf[100];
    sprintf(buf, "%02d %02d %02d %02d %02d %02d", location[0], location[1], location[2], location[3], location[4], location[5] );
    print_str(buf, 1);
    sprintf(buf, "%02d %02d %02d %02d %02d %02d", location[6], location[7], location[8], location[9], location[10], location[11] );
    print_str(buf, 2);
}


void halt(void) {
    while (ev3_button_is_pressed(ENTER_BUTTON) == false) {}
}



#if 1
void steering(int power, int cm, int steering) {
    int b_counts;
    int c_counts;
    cm = abs(cm);
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    power = -power;
    ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steering);
    while (true) {
        b_counts = abs(ev3_motor_get_counts(EV3_PORT_B));
        c_counts = abs(ev3_motor_get_counts(EV3_PORT_C));
        if (cm * ROBOT1CM < b_counts || cm * ROBOT1CM < c_counts) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}
#endif

/*
    distance move forward with steering.
    台形制御あり
    arg:
        power : maximum motor power
        cm : distance in centimeter
        steering : steering 
    retuen:
        none

*/


#if 1
float min_power = 15.0;
float thres_distance = 0.5;
float steer_adjust = -0.07;    //   8.2-8.3Vの間で調整  0.0　-　-0.6: 左に寄る、-0.7 - -1.0　右に寄る。

/* 直進　三角加速/減速　*/
void forward(int power, int cm) {
    int b_counts;
    int c_counts;
    float target_count = (float)abs(cm) * ROBOT1CM;
    int target_power = abs(power);
    int dir = 0;

    if (power >= 0) dir = -1;
    else            dir = 1;
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);

    float p; // current power
    int pwr;

    while (1) {
        b_counts = abs(ev3_motor_get_counts(EV3_PORT_B));
        c_counts = abs(ev3_motor_get_counts(EV3_PORT_C));

        int dCount = b_counts - c_counts;
        int steer = (int)floor((float)dCount * steer_adjust + 0.5); 

        float count = (float)(b_counts + c_counts) / 2.0;
        if (count < target_count/2.0) {
            p = target_power * count * 2.0 / target_count;
            if (p < min_power) p = min_power;
        } else {
            p = target_power * 2.0 - (target_power * count * 2.0 / target_count);
            if (p > 0 && p < min_power) p = min_power;
            if (p < 0 && p > -min_power) p = -min_power;
        } 
        if (abs(count - target_count) < thres_distance) break;
        pwr = (int)floor(p+0.5);
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, pwr * dir, steer);

    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);

    char buf[100];
    sprintf(buf, "B: %d", b_counts);
    print_str(buf, 1);
    sprintf(buf, "C: %d", c_counts);
    print_str(buf, 2);
    sprintf(buf, "battery: %d mV", volt());
    print_str(buf, 3);

}

#endif

// float min_power = 15.0;
float thres_turn = 0.1;
float turn_adjust = -0.0;

void my_turn(int power, int angle) {
    int b_counts;
    int c_counts;
    float target_count = (abs)((float)angle*TURN*ROBOT1CM);
    int target_power = power;

    int dir = 0;                            //  方向
    if (angle >= 0)    dir = 1;
    else                dir = -1;

    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);

    float p; // current power
    int pwrL, pwrR;

    while (1) {
        b_counts = abs(ev3_motor_get_counts(EV3_PORT_B));
        c_counts = abs(ev3_motor_get_counts(EV3_PORT_C));

        char buf[100];
        sprintf(buf, "B: %d", b_counts);
        print_str(buf, 1);
        sprintf(buf, "C: %d", c_counts);
        print_str(buf, 2);

        int dCount = b_counts + c_counts;
        int dTurn = (int)floor((float)dCount * turn_adjust + 0.5); 

        float count = (float)(abs(b_counts) + abs(c_counts)) / 2.0;
        if (count < target_count/2.0) {
            p = target_power * count * 2.0 / target_count;
            if (p < min_power) p = min_power;
        } else {
            p = target_power * 2.0 - (target_power * count * 2.0 / target_count);
            if (p > 0 && p < min_power) p = min_power;
            if (p < 0 && p > -min_power) p = -min_power;
        } 
        if (abs(count - target_count) < thres_turn) break;
        if (p > 0) {
            pwrL = (int)floor(p+0.5);
            pwrR = (int)floor(p+dTurn+0.5);
        } else {
            pwrL = pwrR = -(int)floor(abs(p)+0.5);
        }
//        ev3_motor_rotate(EV3_PORT_B, angle*TURN*ROBOT1CM, +pwr, true);
//        ev3_motor_rotate(EV3_PORT_C, angle*TURN*ROBOT1CM, -pwr, true);
        ev3_motor_set_power(EV3_PORT_B, pwrL * dir);
        ev3_motor_set_power(EV3_PORT_C, -pwrR * dir);
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}


void test_square(void) {

    forward(90, 50);
    tslp_tsk(1200*MSEC);

    my_turn(70, 90);
    tslp_tsk(1200*MSEC);

    forward(90, 50);
    tslp_tsk(1200*MSEC);

    my_turn(70, 90);
    tslp_tsk(1200*MSEC);

    forward(90, 50);
    tslp_tsk(1200*MSEC);

    my_turn(70, 90);
    tslp_tsk(1200*MSEC);

    forward(90, 50);
    tslp_tsk(1200*MSEC);

    my_turn(70, 90);
    tslp_tsk(1200*MSEC);

}


void test_forward(void) {

    forward(90, 50);
    tslp_tsk(1200*MSEC);

    forward(90, 50);
    tslp_tsk(1200*MSEC);

    forward(90, 50);
    tslp_tsk(1200*MSEC);

    forward(90, 50);
    tslp_tsk(1200*MSEC);

}

void test_turn(void) {

    my_turn(35, 90);                // power が小さいほど正確な気がする。 
    tslp_tsk(500*MSEC);

    my_turn(35, 90);
    tslp_tsk(500*MSEC);

    my_turn(35, 90);
    tslp_tsk(500*MSEC);

    my_turn(35, 90);
    tslp_tsk(500*MSEC);
}


void test_backward(void) {

    forward(-90, 50);
    tslp_tsk(500*MSEC);

    forward(-90, 50);
    tslp_tsk(500*MSEC);

    forward(-90, 50);
    tslp_tsk(500*MSEC);

    forward(-90, 50);
    tslp_tsk(500*MSEC);

}



#if 0
float start_power = 15.0;
float end_power = 30.0;
float dPower = 0.5;

void steering(int power, int cm, int steering) {
    int b_counts;
    int c_counts;
    float target_count = (float)abs(cm) * ROBOT1CM;
    float half_count = target_count / 2.0;
    int target_power = power;

    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);

    float p = start_power;    // current power in calicuration
    int pwr;    // current power to be set
    float last_count = 0;

    ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, -(int)p, 3);
    while (1) {
        b_counts = abs(ev3_motor_get_counts(EV3_PORT_B));
        c_counts = abs(ev3_motor_get_counts(EV3_PORT_C));
        float count = (float)(b_counts + c_counts) / 2.0;
        if (count < last_count+5) continue;                // 以下は、5度おきに処理する。
        last_count = count;
        if (count < half_count) {
            p += dPower;
        } else {
            p -= dPower;
            if (p < end_power) p = end_power; 
        } 
        if (count >= target_count) break;
        pwr = (int)floor(p);
        if (pwr > target_power) pwr = target_power;
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, -pwr, steering);
    }

    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}


#endif






/*
void newsteering(int power, float cm) {
    int set_power = -power;
    int p;
    int gyro;
    float steer;
    int left;
    int right;
    int difference;
    float accele_length = cm / 10 * 0;
    float maxspeed_length = cm / 10 * 10;
    float decele_length = cm;

    int left_diff = 0;
    int left_last = 0;
    int right_diff = 0;
    int right_last = 0;
    int average = 0;

    
    
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
        left_last = left;
        left = ev3_motor_get_counts(EV3_PORT_B);
        left_diff = left - left_last;

        right_last = right;
        right = ev3_motor_get_counts(EV3_PORT_C);
        right_diff = right - right_last;

        average = (left_diff + right_diff) / 2;

        difference = left - right;
        float diff = sin(gyro) * average;
        y = y + diff;
        float coef_diff = 0.0;
        float coef_gyro = 0;
        p = -(gyro * 5 + y * coef_gyro);
        steer = p;
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, set_power, steer);
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
        if (power > -20 && set_power < 0) power = -20;
        if (power < 20 && set_power > 0) power = 20;
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
        if (decele_length * ROBOT1CM <= left) break;
    }
    ev3_speaker_play_tone(NOTE_A4, 200);
    tslp_tsk(1000);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}*/

void location_perfect(int power, float cm_1st, float cm_2nd, float cm_3rd, float cm_4th, float map_1st, float map_2nd, float map_3rd, float map_4th, float way_1st, float way_2nd, float way_3rd, float way_4th) {
    
    
    
    
    float cm = cm_1st + cm_2nd + cm_3rd + cm_4th; 
    int set_power = -power;
    int p_gyro;
    int p_diff;
    int gyro;
    int left;
    int right;
    int difference;
    float accele_length = cm / 10 * 1;
    float maxspeed_length = cm / 10 * 9;
    float decele_length = cm / 10 * 1;
    float steer;
    
    
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
        if (set_power > 0) p_gyro = gyro;
        else p_gyro = -gyro;
        p_diff = -difference;
        if (accele_length * ROBOT1CM > left) {
            steer = p_diff * 6 + p_gyro * 4;
            power = (set_power / accele_length) * (left / ROBOT1CM);
            if (power > -15 && set_power < 0) power = -15;
            if (power < 15 && set_power > 0) power = 15;
            ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
        }
        else if (maxspeed_length * ROBOT1CM > left) {
            steer = p_diff * 5 + p_gyro * 4;
            ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
        }
        else {
            steer = p_diff * 6 + p_gyro * 4;
            power = (-set_power / decele_length) * ((left / ROBOT1CM ) - maxspeed_length) + set_power;
            if (power > -10 && set_power < 0) power = -10;
            if (power < 10 && set_power > 0) power = 10;
            ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
        }
        if (cm * ROBOT1CM <= left) break;
        if (cm_1st*MSEC <= left && way_1st == RIGHT && task_counts == 0) sta_cyc(LOCATION_R_TASK);
        if (cm_1st*MSEC <= left && way_1st == LEFT && task_counts == 0) sta_cyc(LOCATION_L_TASK);
        task_counts = 0;
        if (cm_2nd*MSEC <= left && way_2nd == RIGHT && task_counts == 0) sta_cyc(LOCATION_R_TASK);
        if (cm_2nd*MSEC <= left && way_2nd == LEFT && task_counts == 0) sta_cyc(LOCATION_L_TASK);
        task_counts = 0;
        if (cm_3rd*MSEC <= left && way_3rd == RIGHT && task_counts == 0) sta_cyc(LOCATION_R_TASK);
        if (cm_3rd*MSEC <= left && way_3rd == LEFT && task_counts == 0) sta_cyc(LOCATION_L_TASK);
        task_counts = 0;
        if (cm_4th*MSEC <= left && way_4th == RIGHT && task_counts == 0) sta_cyc(LOCATION_R_TASK);
        if (cm_4th*MSEC <= left && way_4th == LEFT && task_counts == 0) sta_cyc(LOCATION_L_TASK);
        task_counts = 0;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void calculate(int map, int place_saves) {
    savedate_color[place_saves] = obj;
    savedate_red[place_saves] = red;
    savedate_green[place_saves] = green;
    savedate_blue[place_saves] = blue;
    judgement = red + green + blue;
    location_sensor[map] = obj;
    switch (obj){
        case 1:
        case 2:
        case 3:
        case 4:
        case 11:
        case 13:
        case 16:
            location[map] = PERSON;
            break;
        case 12:
        case 14:
        case 17:
            
            if (judgement > 80 || blue > 40 || green > 40 || red > 40) {
                location[map] = PERSON;
            } 
            else {
                judgement_check = judgement;
                location[map] = NOTHING;
            }
            break;
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 15:
            location[map] = FIRE;
            break;
        case 0:
            if (red > 15 && red > blue && red > green) location[map] = FIRE;
            else location[map] = CHEMICAL;
            break; 
        default:
            location[map] = NOTHING; 
    }
    fprintf(bt, "LOCATION = %d\r\nCOLOR = %d  RGB:%d,%d,%d = JUDGE:%d\r\nRESULT = %d\r\n-----------------\r\n", map, obj, red, green, blue, judgement, location[map]);
}


void newsteering(int power, float cm) {
    int set_power = -power;
    int p_gyro;
    int p_diff;
    int gyro;
    int left;
    int right;
    int difference;
    float accele_length = cm / 10 * 1;
    float maxspeed_length = cm / 10 * 9;
    float decele_length = cm / 10 * 1;
    float steer;
    
    
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
        if (set_power > 0) p_gyro = gyro;
        else p_gyro = -gyro;
        p_diff = -difference;
        steer = p_diff * 6 + p_gyro * 4;
        power = (set_power / accele_length) * (left / ROBOT1CM);
        if (power > -15 && set_power < 0) power = -15;
        if (power < 15 && set_power > 0) power = 15;
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
        if (set_power > 0) p_gyro = gyro;
        else p_gyro = -gyro;
        p_diff = -difference;
        steer = p_diff * 5 + p_gyro * 4;
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
        if (set_power > 0) p_gyro = gyro;
        else p_gyro = -gyro;
        p_diff = -difference;
        steer = p_diff * 6 + p_gyro * 4;
        power = (-set_power / decele_length) * ((left / ROBOT1CM ) - maxspeed_length) + set_power;
        /*if (power < 20 && power >= 0 && set_power > 0) power = 20;
        if (power > -20 && power <= 0 && set_power < 0) power = -20;*/
        if (power > -10 && set_power < 0) power = -10;
        if (power < 10 && set_power > 0) power = 10;
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, steer);
        if (cm * ROBOT1CM <= left) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    
}

void newsteering_test(int power, float cm) {
    int set_power = -power;
    int p_gyro;
    int p_diff;
    int gyro;
    int left;
    int right;
    int difference;
    float accele_length = cm / 10 * 1;
    float maxspeed_length = cm / 10 * 9;
    float decele_length = cm / 10 * 1;
    float steer;
    
    
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
        if (set_power > 0) p_gyro = gyro;
        else p_gyro = -gyro;
        p_diff = -difference;
        steer = p_diff * 6 + p_gyro * 4;
        power = (set_power / accele_length) * (left / ROBOT1CM);
        if (power > -15 && set_power < 0) power = -15;
        if (power < 15 && set_power > 0) power = 15;
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, 0);
        if (accele_length * ROBOT1CM <= left) break;
    }
    while(true){
        gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        difference = left - right;
        if (set_power > 0) p_gyro = gyro;
        else p_gyro = -gyro;
        p_diff = -difference;
        steer = p_diff * 5 + p_gyro * 4;
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, 0);
        if (maxspeed_length * ROBOT1CM <= left) break;
    }
    while(true){
        gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        left = ev3_motor_get_counts(EV3_PORT_B);
        right = ev3_motor_get_counts(EV3_PORT_C);
        left = abs(left);
        right = abs(right);
        difference = left - right;
        if (set_power > 0) p_gyro = gyro;
        else p_gyro = -gyro;
        p_diff = -difference;
        steer = p_diff * 6 + p_gyro * 4;
        power = (-set_power / decele_length) * ((left / ROBOT1CM ) - maxspeed_length) + set_power;
        /*if (power < 20 && power >= 0 && set_power > 0) power = 20;
        if (power > -20 && power <= 0 && set_power < 0) power = -20;*/
        if (power > -10 && set_power < 0) power = -10;
        if (power < 10 && set_power > 0) power = 10;
        ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, power, 0);
        if (cm * ROBOT1CM <= left) break;
    }
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
    angle = angle * 178 / 180;
    int turn_check = 0;
    int gyro = 0;
    int power;
    float accele_length = angle / 10 * 1;
    float maxspeed_length = angle / 10 * 2;
    ev3_gyro_sensor_reset(EV3_PORT_4);
    gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
    gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
    while (true) {
        gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        gyro = abs(gyro);
        power = (100 / (angle / 10 * 1)) * gyro;
        if(power < 20) power = 20;
        if(power > 80) power = 80;
        if (gyro > accele_length) break;

        if (left_motor == 0) {
            power = power * 1.3;
            (void)ev3_motor_set_power(EV3_PORT_C, -power * right_motor); 
        }
        else if (right_motor == 0) {
            power = power * 1.3;
            (void)ev3_motor_set_power(EV3_PORT_B, -power * left_motor);
        } 
        else {
            (void)ev3_motor_set_power(EV3_PORT_B, -power * left_motor);
            (void)ev3_motor_set_power(EV3_PORT_C, -power * right_motor); 
        }
    }
    while (true) {
        gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        gyro = abs(gyro);
        power = 80;
        if (gyro > maxspeed_length) break;

        if (left_motor == 0) {
            power = power * 1.3;
            (void)ev3_motor_set_power(EV3_PORT_C, -power * right_motor); 
        }
        else if (right_motor == 0) {
            power = power * 1.3;
            (void)ev3_motor_set_power(EV3_PORT_B, -power * left_motor);
        } 
        else {
            (void)ev3_motor_set_power(EV3_PORT_B, -power * left_motor);
            (void)ev3_motor_set_power(EV3_PORT_C, -power * right_motor); 
        }
    }
    while (true) {
        gyro = ev3_gyro_sensor_get_angle(EV3_PORT_4);
        gyro = abs(gyro);
        power = (angle - gyro) * 1;
        if (angle - gyro == 0) turn_check = turn_check + 1;

        if (left_motor == 0) {
            power = power * 1.3;
            if(power < 6 && power > 0) power = 6;
            if(power > -6 && power < 0) power = -6;
            (void)ev3_motor_set_power(EV3_PORT_C, -power * right_motor); 
        }
        else if (right_motor == 0) {
            power = power * 1.3;
            if(power < 6 && power > 0) power = 6;
            if(power > -6 && power < 0) power = -6;
            (void)ev3_motor_set_power(EV3_PORT_B, -power * left_motor);
        } 
        else {
            if(power < 6 && power > 0) power = 6;
            if(power > -6 && power < 0) power = -6;
            (void)ev3_motor_set_power(EV3_PORT_B, -power * left_motor);
            (void)ev3_motor_set_power(EV3_PORT_C, -power * right_motor); 
        }
        if (turn_check > 500) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    tslp_tsk(100*MSEC);

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
    ev3_motor_reset_counts(EV3_PORT_B);
    int kakudo_C; 
    int kakudo_B; 
    float steer;
    float p = 0;
    float i = 0;
    float d = 0;
    float d2 = 0;
    int reflect;
    power = -power;
    while (true) {
        kakudo_C = ev3_motor_get_counts(EV3_PORT_C);
        kakudo_B = ev3_motor_get_counts(EV3_PORT_B);
        reflect = ev3_color_sensor_get_reflect(EV3_PORT_1);
        
        p = reflect - 35;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  -p * P_GEIN + i * 0 + d * 0; 
        if (length * ROBOT1CM < -(kakudo_C + kakudo_B) / 2) break;
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
        while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_3, &test)) {
            ;
        }  
        while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_3, &obj)) {
            ;
        }  
        while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_3, &testrgb)) {
                ;
        }  
        while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_3, &rgb_val)) {
                ;
        }  
        tslp_tsk(100);
        
    } 
    else{ 
        while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_2, &test)) {
            ;
        }  
        while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_2, &obj)) {
            ;
        }  
        while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_2, &testrgb)) {
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
        case 2:
        case 3:
        case 4:
        case 11:
        case 13:
        case 16:
            location[num] = PERSON;
            play_wave(WAVE_Shiro);
            break;
        case 1:
        case 12:
        case 14:
        case 17:
            
            if (judgement > 80 || blue > 40 || green > 40 || red > 40) {
                location[num] = PERSON;
                if (red - green - blue > 20) { location[num] = FIRE;  play_wave(WAVE_Aka); }
                else { play_wave(WAVE_Shiro); }

    
            } 
            else {
                judgement_check = judgement;
                location[num] = NOTHING;
                play_wave(WAVE_Nashi);
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
            play_wave(WAVE_Aka);
            break;
        case 0:
            if (red > 15 && red > blue && red > green) {
                location[num] = FIRE;
                play_wave(WAVE_Aka);
            }
            else {
                location[num] = CHEMICAL;
                play_wave(WAVE_Kuro);
            }
            break; 
        default:
            location[num] = NOTHING;
            play_wave(WAVE_Nashi);
            break;
    } 
    fprintf(bt, "LOCATION = %d\r\nCOLOR = %d  RGB:%d,%d,%d = JUDGE:%d\r\nRESULT = %d\r\n-----------------\r\n", num, obj, red, green, blue, judgement, location[num]);
    /*ev3_speaker_play_tone(NOTE_A5, 100);
    tslp_tsk(400*MSEC);*/
}

void hsv(int num, way_t sensor) {
    /*ev3_speaker_play_tone(NOTE_A5, 100);
    tslp_tsk(300*MSEC);*/
    float h = 0;
    float s = 0;
    float v = 0;
    float min = 0;
    float max = 0;
    float red;
    float green;
    float blue;
    float judgement;
    while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_2, &test)) {
        ;
    }  
    while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_2, &obj)) {
        ;
    }  
    while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_2, &testrgb)) {
            ;
    }  
    while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_2, &rgb_val)) {
            ;
    }  
    
    red = rgb_val.r;
    green = rgb_val.g;
    blue = rgb_val.b;
    judgement = red + green + blue;

    if (red <= green && red <= blue) { 
        min = red;
    }
    if (green <= red && green <= blue) { 
        min = green;
    }
    if (blue <= red && blue <= green) { 
        min = blue;
    }
    
    if (red >= green && red >= blue) { 
        max = red;
        h = 60 * ((green - blue) / (max - min));
    }
    if (green >= red && green >= blue) { 
        max = green;
        h = 60 * ((blue - red) / (max - min)) + 120;
    }
    if (blue >= red && blue >= green) { 
        max = blue;
        h = 60 * ((red - green) / (max - min)) + 240;
    }

    s = (max - min) / max * 100;

    v = max / 255 * 100;

    location_sensor[num] = obj;

    fprintf(bt, "LOCATION = %d\r\nCOLOR = %d  RGB:%f,%f,%f = JUDGE:%f\r\nRESULT = %d\r\nH:%f\r\nS:%f\r\nV:%f\r\nmax:%f\r\nmin:%f\r\n-----------------\r\n", num, obj, red, green, blue, judgement, location[num], h, s, v, max, min);
    /*ev3_speaker_play_tone(NOTE_A5, 100);
    tslp_tsk(400*MSEC);*/
}


void arm_up(void) {
    if (isArmUp) return;                            // 既にup状態だったら何もしない
    ev3_motor_rotate(EV3_PORT_A, 200, 15, false);
    isArmUp = true;
}




void arm_down(void) {
    if (!isArmUp) return;                            // 既にdown状態だったら何もしない
    ev3_motor_rotate(EV3_PORT_A, 200, -15, false);
    isArmUp = false;
}

void chemical_taker(int n, way_t sensor){
    if(location[n] == CHEMICAL) {
        if (isArmUp) return;            // もうarmがあがっていたら、帰る。左右どちらかの情報を上書きしない。
        chemical = chemical + 1;
        arm_up();
        chemical_type = sensor;
    }
}

void chemical_took(int n, way_t sensor){
    if(location[n] == CHEMICAL){
       if(sensor == RIGHT){
           ev3_motor_rotate(EV3_PORT_A, 30, -15, false);
           chemical_type = RIGHT;
        }
        else{
            ev3_motor_rotate(EV3_PORT_A, 30, 15, false);
            chemical_type = LEFT;
        }
    }
}

void water(int n) {
    if (location[n] == FIRE) {
        ev3_motor_rotate(EV3_PORT_D, 80 + water_count, 25, false);
        water_count = water_count + 30;
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

void check_task(intptr_t unused){
    fprintf(bt, "\r\ny:%d", y);
}

void location_r_task(intptr_t unused){
    stp_cyc(LOCATION_R_CYC);
    while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_3, &test)) {
        ;
    }  
    while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_3, &obj)) {
        ;
    }  
    while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_3, &testrgb)) {
            ;
    }  
    while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_3, &rgb_val)) {
            ;
    }  
    savedate_color[location_counts] = obj;
    savedate_red[location_counts] = rgb_val.r;
    savedate_green[location_counts] = rgb_val.g;
    savedate_blue[location_counts] = rgb_val.b;
    location_counts = location_counts + 1;
    task_counts = task_counts + 1;
}

void location_l_task(intptr_t unused){
    stp_cyc(LOCATION_L_CYC);
    while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_2, &test)) {
        ;
    }  
    while ( ! ht_nxt_color_sensor_measure_color(EV3_PORT_2, &obj)) {
        ;
    }  
    while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_2, &testrgb)) {
            ;
    }  
    while ( ! ht_nxt_color_sensor_measure_rgb(EV3_PORT_2, &rgb_val)) {
            ;
    }  
    savedate_color[location_counts] = obj;
    savedate_red[location_counts] = rgb_val.r;
    savedate_green[location_counts] = rgb_val.g;
    savedate_blue[location_counts] = rgb_val.b;
    location_counts = location_counts + 1;
    task_counts = task_counts + 1;
}


void aproach_to_brown(void) {
    steering_color(COLOR_WHITE, 35, 0);     // 白ゾーンまで進む
    steering_color(COLOR_BLACK, 35, 0);     // 黒線を見つけるまえすすむ
    forward(60, 2.0);

    // ライントレース
    beep();
    linetrace_length(28.5, 9);
    // 最初のオブジェクトの横に進む
    forward(60, 4.1);           // location 8まで進む
    tslp_tsk(500*MSEC);
    
    ev3_motor_reset_counts(EV3_PORT_D);
}

void brown_zone(void) {

    /* brown zone */    
    map_check(8, RIGHT);
    chemical_taker(8, RIGHT);
    tslp_tsk(600*MSEC);         // takerが動いている間待つこと
    forward(90, 36.7);
    map_check(9, RIGHT);
    chemical_taker(9, RIGHT);
    water(8);
    water(9);
    tslp_tsk(600*MSEC);         // takerが動いている間待つこと
    forward(90, 49);            // loocation 10,11まで進む
}

void red_zone(void) {
    /* red zone */    
    map_check(10, RIGHT);
    tslp_tsk(600*MSEC);         // wave再生のあいだ待つ
    map_check(11, LEFT);
    steering_time(500, 40, 0);  // 壁にあてて
    forward(-45, 8.0);          // 　バックする

    if (location[10] == CHEMICAL){
        my_turn(45, -90);
        chemical_taker(10, LEFT);
        tslp_tsk(500*MSEC);
        steering_time(1200, -25, 0);    // 壁にあてて 
        forward(90, 12.5);
    } else if (location[11] == CHEMICAL){
        my_turn(45, 90);
        chemical_taker(11, RIGHT);
        tslp_tsk(1000*MSEC);
        steering_time(800, 25, 0);    // 壁にあてて 
        forward(-60, 4.5);              // ばっくして
        my_turn(45, 180);               // 180度旋回してむきをかえる
    } else {
        my_turn(45, -90);
        steering_time(1200, -25, 0);    // 壁にあてて 
        forward(90, 12.5);
    }
    // この時点で、location 10の位置に止まっている。
    water(10);
    water(11);
    forward(90, 38);            // location 4まで進む
}


void yellow_zone(void) {
    // ここでは、location 7はチェックしない
    my_turn(45, -90);
    steering_time(1200, -30, 0);
    forward(70, 12.5);
    my_turn(45, 90);

    map_check(4, LEFT);         // 
    chemical_taker(4, LEFT);
    water(4);
    tslp_tsk(500*MSEC);
    forward(70, 39.2);      // localtion 3 まで進む
    steering_time(500, 30, 0);  // 正確を期すため、一度壁に当てて戻る
    forward(-70, 4.5);
    tslp_tsk(500*MSEC);
}

void green_zone(void) {
    /* green zone */
    map_check(3, LEFT);
    chemical_taker(3, LEFT);
    tslp_tsk(600*MSEC);
    my_turn(45, -90);               //
    steering_time(800, -30, 0);     // 壁にあてて
    forward(90, 22);              // location 2 まで進む
    map_check(2, LEFT);
    chemical_taker(2, LEFT);
    tslp_tsk(600*MSEC);
    water(2);
    water(3);
    forward(90, 11);      // location 1に進む
}

void blue_zone(void) {
    map_check(1, LEFT);
    chemical_taker(1, LEFT);
    tslp_tsk(600*MSEC);
    forward(90, 37.8);
    map_check(0, LEFT);
    chemical_taker(0, LEFT);
    tslp_tsk(600*MSEC);
}

int guess_white(void) {
    return how_many - (location[0] + location[1] + location[2] + location[3] + location[4] + location[7] + location[8] + location[9] + location[10] + location[11]);
}



void clap_marker(int left, int right) {
    if (right == 1) {
        ev3_motor_rotate(EV3_PORT_D, 50, 40, true); /*時計回りがプラス*/
        tslp_tsk(300*MSEC);
        forward(45, 3.2);
        ev3_motor_rotate(EV3_PORT_D, 70, -25, true);
        tslp_tsk(300*MSEC);
        ev3_motor_rotate(EV3_PORT_D, 20, 40, true);
        tslp_tsk(300*MSEC);
        forward(45, 3.0);
    }
    else forward(45, 7.4);
    if (left == 1) {
        ev3_motor_rotate(EV3_PORT_D, 50, -40, true); 
        tslp_tsk(300*MSEC);
        forward(45, 3.2);
        ev3_motor_rotate(EV3_PORT_D, 70, 30, true);
        tslp_tsk(300*MSEC);
        ev3_motor_rotate(EV3_PORT_D, 20, -40, true);
        tslp_tsk(300*MSEC);
        forward(45, 3.0);
    }
    else forward(45, 8.7);
}


void marker_goal_1(void) {
#if 0
    map[0] = 1; /*blue*/
    map[1] = 0; /*green*/
    map[2] = 1; /*white*/
    map[3] = 0; /*yellow*/
    map[4] = 0; /*brown*/
    map[5] = 0; /*red*/
#endif

    tank_turn(185, -30, 0);             // start/goal 2 を向いて、brownに入る
    tslp_tsk(400*MSEC);
    tank_turn(184, 0, -30);
    tslp_tsk(200*MSEC);
    newsteering(70, 25.0);

    if (map[4] == 1 || map[5] == 1) {
        clap_marker(map[5], map[4]);
    }
    else newsteering(50, 14.3);

    if (map[2] == 1 || map[3] == 1) {
        clap_marker(map[3], map[2]);
    }
    else newsteering(50, 14.3);

    if (map[0] == 1 || map[1] == 1) {
        clap_marker(map[1], map[0]);
    }
    else newsteering(50, 14.3);

    steering_time(1500, 30, -40);       // ゴールエリアに入り込む

}




void main_task(intptr_t unused){

    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    ev3_lcd_set_font(EV3_FONT_MEDIUM);
    ev3_font_get_size(EV3_FONT_MEDIUM, &fontw, &fonth);

    ev3_speaker_set_volume(10);
    beep();
    /* Register button handlers */
    ev3_button_set_on_clicked(BACK_BUTTON, &button_clicked_handler, BACK_BUTTON);

    init_wave();

    /* Configure motors */
    ev3_motor_config(PortMotorLeft, LARGE_MOTOR);
    ev3_motor_config(PortMotorRight, LARGE_MOTOR);
    ev3_motor_config(PortMotorArm1, LARGE_MOTOR);
    ev3_motor_config(PortMotorArm2, LARGE_MOTOR);



    /* Configure sensors */

    ev3_sensor_config(PortSensorGYRO, GYRO_SENSOR);
    ev3_sensor_config(PortSensorColor1, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor2, HT_NXT_COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor3, HT_NXT_COLOR_SENSOR);


    ev3_lcd_set_font(EV3_FONT_SMALL);

    while ( !ht_nxt_color_sensor_measure_color(EV3_PORT_3, &test) ) {
            ;
        }
    while ( !ht_nxt_color_sensor_measure_color(EV3_PORT_2, &test) ) {
            ;
        }   
    ev3_color_sensor_get_color(EV3_PORT_1);

    print_str("SELECT PROGRAM", 1);
    print_str("ENTER : MISSION", 2);
    print_str("LEFT : 90deg. TURN TEST", 3);
    fprintf(bt, "----GAME_START----\r\n");

    halt();
    beep();
    tslp_tsk(500*MSEC);

/*
    red_zone();
    yellow_zone();
    green_zone();
    blue_zone();
    water(0);
    water(1);
    tslp_tsk(1000*MSEC);

    halt();
    while (1) {
    marker_goal_1();
    halt();
    }
*/

    /*ここからコーディング */
/*
    aproach_to_brown();

    brown_zone();
    halt();

    red_zone();
    halt();

    yellow_zone();
    halt();

    green_zone();
    halt();

    blue_zone();
    halt();
*/

    /*sensor
    0 = Black, 1 = Purple, 2 =PuBl, 3 = Brue, 4 = Green, 5 = GrYe,
    6 = Yellow, 7 = Orange, 8 = Red, 9 = PiRe, 10 = Pink, 
    11 = WhBl, 12 = WhGr, 13 = WhYe, 14 = WhOr, 15 = WhRe, 16 = WhPi,
    17 = White*/


    /*スタートの分岐チェック*/

   
    start = 1;
    
    tslp_tsk(500*MSEC);

    aproach_to_brown();
    
    /* 　ここから工場内　*/

    brown_zone();
    red_zone();
    yellow_zone();
    green_zone();
    blue_zone();

    // 以下の数式により、white zoneのオブジェクトが類推される。
    white = guess_white();
    
    if (white == CHEMICAL){
        newsteering(-35, 6);            // すこしバックして
        tslp_tsk(500*MSEC);
        p_turn(90, 1, -1);              // 壁に当ててwhite zoneに向かう
        steering_time(1200, -30, 0);
        forward(70, 5.0);
        water(0);                       // 途中、waterを落とす
        water(1);
        newsteering(70, 44.7);          // white zoneに入る
        tslp_tsk(500*MSEC);
        p_turn(90, -1, 1);
        tslp_tsk(200*MSEC);
        newsteering(-40, 32);
        map_check(6, RIGHT);
        chemical_taker(6, RIGHT);
        newsteering(60, 37);
        map_check(5, RIGHT);
        chemical_taker(5, RIGHT);
        newsteering(70, 69);
        tslp_tsk(200*MSEC);
        p_turn(90, 1, -1);
        tslp_tsk(200*MSEC);
        newsteering(-30, 8);
    }    
    else if (white == FIRE){
        location[5] = FIRE;
        newsteering(-30, 6);
        water(0);
        water(1);
        tslp_tsk(700*MSEC);
        p_turn(90, -1, 1);
        steering_time(700, 30, 0);
        newsteering(-70, 38);
        tslp_tsk(400*MSEC);
        p_turn(90, 1, -1);
        water(5);
        tslp_tsk(700*MSEC);
        newsteering(80, 76);
        tslp_tsk(200*MSEC);
        p_turn(90, 1, -1);
        newsteering(-30, 8);
    }
    else if (white == PERSON || white == NOTHING) {
        if (white == PERSON) location[5] = PERSON;
        else location[5] = NOTHING;
        water(0);
        water(1);
        newsteering(80, 68);
        tslp_tsk(200*MSEC);
        p_turn(90, 1, -1);
        steering_time(1500, -30, 0);
        newsteering(70, 40);
    }
    else {
        newsteering(-35, 6);
        tslp_tsk(500*MSEC);
        p_turn(90, 1, -1);
//        if(chemical < 1)ev3_motor_rotate(EV3_PORT_A, 280, 20, false);    armUp() では不要
        steering_time(1200, -30, 0);
        forward(70, 5.0);
        water(0);
        water(1);
        newsteering(70, 44.7);
        tslp_tsk(500*MSEC);
        p_turn(89, -1, 1);
        tslp_tsk(200*MSEC);
        newsteering(30, 5);
        map_check(5, RIGHT);
        chemical_taker(5, RIGHT);
        newsteering(-40, 37);
        map_check(6, RIGHT);
        chemical_taker(6, RIGHT);
        newsteering(70, 102);
        tslp_tsk(200*MSEC);
        p_turn(90, 1, -1);
        tslp_tsk(200*MSEC);
        newsteering(-30, 8);
    }

    // ケミカルを置く
    if(chemical_type == RIGHT){
        newsteering(50, 24);
        my_turn(45, -180);
        arm_down();
        tslp_tsk(1000 * MSEC);
        my_turn(45, 180);    
    }
    else{
        arm_down();
        tslp_tsk(1000 * MSEC);
    }
    
    // マーカアームを右に水平にセットする
    if (ev3_motor_get_counts(EV3_PORT_D) > 90) {
        while(true){
            ev3_motor_set_power(EV3_PORT_D, -20);
            if (ev3_motor_get_counts(EV3_PORT_D) < 90)break;
        }
        ev3_motor_stop(EV3_PORT_D, true);
    }
    if (ev3_motor_get_counts(EV3_PORT_D) < 90) {
        while(true){
            ev3_motor_set_power(EV3_PORT_D, 20);
            if (ev3_motor_get_counts(EV3_PORT_D) > 90)break;
        }
        ev3_motor_stop(EV3_PORT_D, true);
    }
    ev3_motor_stop(EV3_PORT_D, true);
    
    if(chemical_type == LEFT)newsteering(-70, 46);      // 保管庫の前からバックで壁の直前まで行く
    if(chemical_type == RIGHT)newsteering(-70, 62);

    steering_time(700, -30, 0);                         // 壁に押し付ける
    tslp_tsk(200*MSEC);

    steering_time(200, 30, 0);          // 壁から離れて
    tank_turn(80, 30, 0);               // 壁に寄せながら、おしりをゴールにむける
    tslp_tsk(200*MSEC);
    tank_turn(98, 0, -30);
    steering(-80, 80, 0);               // バックで障害物を超える
    steering_time(2000, -30, 0);        // ゴールの壁に押し付ける
    
    newsteering(50, 17);                // 前に進んで
    tank_turn(140, 0, 45);              // 一旦マーカーの方を向いて
    tslp_tsk(200*MSEC);
    tank_turn(160, 45, -45);            // 旋回して壁に頭をつける
    steering_time(1000, 25, 0);
    
    newsteering(-70, 65);               // バックして一気にマーカーを収納する。
    ev3_motor_rotate(EV3_PORT_D, 90, 30, true); // 
    tslp_tsk(200*MSEC);
    if (location[0] == PERSON || location[1] == PERSON) map[0] = 1; /*blue*/
    if (location[2] == PERSON || location[3] == PERSON) map[1] = 1; /*green*/
    if (location[5] == PERSON || location[6] == PERSON) map[2] = 1; /*white*/
    if (location[4] == PERSON || location[7] == PERSON) map[3] = 1; /*yellow*/
    if (location[8] == PERSON || location[9] == PERSON) map[4] = 1; /*brown*/
    if (location[10] == PERSON || location[11] == PERSON) map[5] = 1; /*red*/
    print_map();

    switch (start) {
    case 1:
        marker_goal_1();
        break;
    
        case 2:
        newsteering(70, 36);
        tslp_tsk(300*MSEC);
        p_turn(180, 1, 0);
        tslp_tsk(300*MSEC);
    
    
        if (map[0] == 1 || map[1] == 1) {
            if (map[1] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, 40, true); /*時計回りがプラス*/
                newsteering(30, 2.2);
                ev3_motor_rotate(EV3_PORT_D, 40, -10, true);
                ev3_motor_rotate(EV3_PORT_D, 10, 40, true);
                newsteering(30, 4.3);
            }
            else newsteering(50, 6.5);
            if (map[0] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, -40, true);
                newsteering(30, 2.6);
                ev3_motor_rotate(EV3_PORT_D, 40, 10, true);
                ev3_motor_rotate(EV3_PORT_D, 10, -40, true);
                newsteering(30, 5.4);
            }
            else newsteering(50, 8);
        }
        else newsteering(50, 14);
    
        if (map[2] == 1 || map[3] == 1) {
            if (map[3] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, 40, true);
                newsteering(30, 2.2);
                ev3_motor_rotate(EV3_PORT_D, 40, -10, true);
                ev3_motor_rotate(EV3_PORT_D, 10, 40, true);
                newsteering(30, 4.3);
            }
            else newsteering(50, 6);
            if (map[2] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, -40, true);
                newsteering(30, 2.6);
                ev3_motor_rotate(EV3_PORT_D, 40, 10, true);
                ev3_motor_rotate(EV3_PORT_D, 10, -40, true);
                newsteering(30, 5.4);
            }
            else newsteering(50, 8);
        }
        else newsteering(50, 14);
    
        if (map[4] == 1 || map[5] == 1) {
            if (map[5] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, 40, true);
                newsteering(30, 2.2);
                ev3_motor_rotate(EV3_PORT_D, 40, -10, true);
                ev3_motor_rotate(EV3_PORT_D, 10, 40, true);
                newsteering(30, 4.3);
            }
            else newsteering(50, 7);
            if (map[4] == 1) {
                ev3_motor_rotate(EV3_PORT_D, 30, -40, true);
                newsteering(30, 2.6);
                ev3_motor_rotate(EV3_PORT_D, 40, 10, true);
                ev3_motor_rotate(EV3_PORT_D, 10, -40, true);
            }
        }
    
        steering_time(3000, 30, 30);

        break;
    }
    
    
    

    


    

}