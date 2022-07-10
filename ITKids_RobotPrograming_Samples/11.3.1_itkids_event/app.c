/** ヘッダファイルの include **/
#include <ev3api.h>
#include "app.h"


/** ファイル内だけで使う関数の宣言 **/
static void prepareDevices(void);
static void cleanUpDevices(void);


/** ファイル内だけで使う定義 **/
#define PORT_SENSOR_TOUCH_LEFT  EV3_PORT_1
#define PORT_SENSOR_TOUCH_RIGHT EV3_PORT_4
#define PORT_SENSOR_COLOR       EV3_PORT_2
#define PORT_SENSOR_ULTRASONIC  EV3_PORT_3
#define PORT_MOTOR_LEFT         EV3_PORT_B
#define PORT_MOTOR_RIGHT        EV3_PORT_C


/** 変数の定義 **/
/* 中央ボタンが押されたら trueとなる変数 */
static bool_t isPressed = false; 


/** すべてのファイルから実行できる関数の定義 **/
/** メインタスク関数 **/
void main_task(intptr_t exinf) {

    /* モーターやセンサーの設定を行う関数を実行する */
    prepareDevices();

    /* ボタンが押されていない状態から開始 */
    isPressed = false;

    /* 中央ボタンが押されていない間はくりかえす  */
    while (false == isPressed) {
        /* 音をならして、LEDを光らせる */
        /* 音を鳴らす関数の実行には時間がかかる */
        (void)ev3_speaker_play_tone(NOTE_C4, 500L);
        (void)ev3_led_set_color (LED_RED);

        /* 500ミリ秒待つ */
        tslp_tsk(500*1000);

        (void)ev3_speaker_play_tone(NOTE_D4, 500L);
        (void)ev3_led_set_color(LED_ORANGE);

        /* 500ミリ秒待つ */
        tslp_tsk(500*1000);

        (void)ev3_speaker_play_tone(NOTE_E4, 500L);
        (void)ev3_led_set_color(LED_GREEN);

        /* 500ミリ秒待つ */
        tslp_tsk(500*1000);

        /* 中央ボタンが押されているか、状態を取り込む */
        isPressed = ev3_button_is_pressed(ENTER_BUTTON);
    }

    /* モーターやセンサーの終了時処理を行う */
    cleanUpDevices();

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
    (void)ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    /* フォントの大きさを EV3_FONT_MEDIUM にする */
    (void)ev3_lcd_set_font(EV3_FONT_MEDIUM);

    /* スピーカーの音量を最大にする */
    (void)ev3_speaker_set_volume(100U);
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
