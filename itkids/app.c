/** ヘッダファイルの include **/
#include <ev3api.h>
#include "app.h"


/** ファイル内だけで使う関数の宣言 **/
static void prepareDevices(void);
static void cleanUpDevices(void);


/** ファイル内だけで使う定義 **/
#define REFLECT_TARGET_VALUE    ((uint8_t)40U)
#define DISTANCE_TARGET_VALUE   ((int16_t)10)
#define POWER_LINETRACE         ((int16_t)30)

#define PORT_SENSOR_TOUCH_LEFT  EV3_PORT_1
#define PORT_SENSOR_TOUCH_RIGHT EV3_PORT_4
#define PORT_SENSOR_COLOR       EV3_PORT_2
#define PORT_SENSOR_ULTRASONIC  EV3_PORT_3
#define PORT_MOTOR_LEFT         EV3_PORT_B
#define PORT_MOTOR_RIGHT        EV3_PORT_C


/** 変数の定義 **/
static int16_t distance = (int16_t)0; 


/** すべてのファイルから実行できる関数の定義 **/
/** メインタスク関数 **/
void main_task(intptr_t exinf) {
    /* この関数(main_task)の中でしか使えない変数 */
    int16_t power = 30;
    int16_t turnRatio = 0;

    int16_t isPressed = false;
    uint8_t reflect = 0U;


    /* モーターやセンサーの設定を行う関数を実行する */
    prepareDevices();

    /* 周期実行タスクを開始 */
   (void)sta_cyc(SENSOR_CYC);

    /* タッチセンサーが押されていない間はくりかえす  */
    while (false == isPressed) {
        /* タッチセンサーから状態を取り込む */
        isPressed = ev3_touch_sensor_is_pressed(PORT_SENSOR_TOUCH_LEFT);

        /* カラーセンサーから反射光強さを得る */
        reflect = ev3_color_sensor_get_reflect(PORT_SENSOR_COLOR);

        /* turnRatio を決める */
        if (reflect < REFLECT_TARGET_VALUE) {
            turnRatio = 50;
        } else if (REFLECT_TARGET_VALUE < reflect) {
            turnRatio = -50;
        } else {
            turnRatio = 0;
        }

        /* powerを決める */
        if (DISTANCE_TARGET_VALUE < distance) {
            power = POWER_LINETRACE;
        } else {
            power = 0; 
        }

        /* モーターに turnRatio を設定する */
        (void)ev3_motor_steer(PORT_MOTOR_LEFT, PORT_MOTOR_RIGHT, power, turnRatio);
    }

    /* モーターやセンサーの終了時処理を行う */
    cleanUpDevices();

    /* 周期実行タスクを終了 */
    (void)stp_cyc(SENSOR_CYC);

    /* 実行中のタスク(main_task)を終了する */
    ext_tsk();
}


/** ファイルの中だけで実行できる関数の定義 **/
static void prepareDevices(void) {
    /* モーターの設定を行う */
    (void)ev3_motor_config(PORT_MOTOR_LEFT, LARGE_MOTOR);
    (void)ev3_motor_config(PORT_MOTOR_RIGHT, LARGE_MOTOR);

    /* センサーの設定を行う */
    (void)ev3_sensor_config(PORT_SENSOR_TOUCH_LEFT, TOUCH_SENSOR);
    (void)ev3_sensor_config(PORT_SENSOR_COLOR, COLOR_SENSOR);
    (void)ev3_sensor_config(PORT_SENSOR_ULTRASONIC, ULTRASONIC_SENSOR);

    /* 画面を消すために、画面いっぱいの白四角を描く */
    (void)ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_BLACK);
    /* フォントの大きさを EV3_FONT_MEDIUM にする */
    (void)ev3_lcd_set_font(EV3_FONT_MEDIUM);



}


static void cleanUpDevices(void) {
    /* モーターを止める */
    (void)ev3_motor_stop(PORT_MOTOR_LEFT, true);
    (void)ev3_motor_stop(PORT_MOTOR_RIGHT, true);

    /* センサーポートを未接続にする */
    (void)ev3_sensor_config(PORT_SENSOR_TOUCH_LEFT, NONE_SENSOR);
    (void)ev3_sensor_config(PORT_SENSOR_COLOR, NONE_SENSOR);
    (void)ev3_sensor_config(PORT_SENSOR_ULTRASONIC, NONE_SENSOR);

    /* 画面を消すために、画面いっぱいの白四角を描く */
    (void)ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
}


void sensor_task(intptr_t exinf) {
    distance = ev3_ultrasonic_sensor_get_distance(PORT_SENSOR_ULTRASONIC);
}
