#include "ev3api.h"
#include "app.h"

static int32_t fontw, fonth;

static uint_t cyc1_count = 0;

static void button_clicked_handler(intptr_t button) {
    switch(button) {
    case BACK_BUTTON:
#if !defined(BUILD_MODULE)
        syslog(LOG_NOTICE, "Back button clicked.");
#endif
        break;
    }
}

/*
    周期タスク
    カラーセンサーを読んで、特定の色だったらサブタスクを起動する。
    他に、LEDを点滅させたり、カラーセンサーの読み取り値を表示している。
*/

void cyc1_led_task(intptr_t unused) {
    if (((++cyc1_count) % 2) == 0) {        // ledを点滅させる。タイマーが動いていることを確認する。
        ev3_led_set_color(LED_GREEN);
    } else {
        ev3_led_set_color(LED_OFF);
    }

    uint8_t test;                                               // カラーセンサー用のバッファ
    ht_nxt_color_sensor_measure_color(EV3_PORT_1, &test);       // カラーセンサーを読む

    ev3_lcd_fill_rect(0, fonth * 1, EV3_LCD_WIDTH, fonth, EV3_LCD_WHITE);   // LCDにカラー値を表示
    char buf[100];
    sprintf(buf, "COLOR = %d", test);
    ev3_lcd_draw_string(buf, 0, fonth * 1);

    if (test == 5) {
        act_tsk(SUB_TASK);             // サブタスクを起動する
    }
}

void armUp(void) {
    ev3_motor_set_power(EV3_PORT_A, 80);    // モーターを回す
    dly_tsk(1000U*1000U);                   // 1秒待つ
    ev3_motor_stop(EV3_PORT_A, true);       //　止める
}

void armDown(void) {
    ev3_motor_set_power(EV3_PORT_A, -80);    // モーターを回す
    dly_tsk(1000U*1000U);                   // 1秒待つ
    ev3_motor_stop(EV3_PORT_A, true);       //　止める
}



/*
    サブタスク                                  
    サブモーターを制御する。    
*/
void sub_task(intptr_t unused) {
    armUp();
}

/*
    サブタスク2
    コマンドで、いくつかの仕事をこなすパターン
*/

int command;
void sub_task2(intptr_t unused) {

    while(true) {

        command = 0;    // commandを初期化（0:無効)にする。
                        // 毎回初期化することで、commandの設定忘れでの誤動作を防ぐ
        slp_tsk();      // タスクをスリープ（待ち状態）させる。
                        // wup_tsk()がコールされると帰ってくる。
                        // 他のタスクで、commandをセットしてwup_tsk()を呼ぶ
                        // while ループになっているので、
        switch (command) {    // commandをチェックして対応した関数を呼び出す。0や、定義していない値では何もしない。
            case 1:
            armUp();    // アームを上げる（未定義）
            break;
            case 2:
            armDown();  // アームを下げる
            break;
        }
    }
}



/*
        メインタスク　- 駆動輪のモーターを制御して全身する。
    */
void main_task(intptr_t unused) {

    /* Register button handlers */
    ev3_button_set_on_clicked(BACK_BUTTON, &button_clicked_handler, BACK_BUTTON);

   /* Configure motors */
    ev3_motor_config(EV3_PORT_A, LARGE_MOTOR);   // サブタスクで回すLモーター
    ev3_motor_config(EV3_PORT_B, LARGE_MOTOR);   // メインタスクで制御：走行用 Lモーター
    ev3_motor_config(EV3_PORT_C, LARGE_MOTOR);   // 　　　　　　　　　　走行用 Lもーたー
    /* Configure sensors */
    ev3_sensor_config(EV3_PORT_1, HT_NXT_COLOR_SENSOR); // カラーセンサー：周期タスクで読む　

    ev3_lcd_set_font(EV3_FONT_MEDIUM);
    ev3_font_get_size(EV3_FONT_MEDIUM, &fontw, &fonth);


    ev3_motor_set_power(EV3_PORT_B, -20);       // power -20でロボットを進める
    ev3_motor_set_power(EV3_PORT_C, -20);

    dly_tsk(5000U * 1000U);                     // 5秒待つ

    ev3_motor_stop(EV3_PORT_B, true);           // ロボットを止める
    ev3_motor_stop(EV3_PORT_C, true);
}
