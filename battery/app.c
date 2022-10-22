#include "ev3api.h"
#include "app.h"

static int32_t fontw, fonth;

static uint_t cyc1_count = 0;
static int isRun = false;
static long time = 0;

FILE *file;//ファイルポインタを宣言
const char* fname = "/battery_result.txt";

static void button_clicked_handler(intptr_t button) {
    switch(button) {
    case ENTER_BUTTON:
        if (isRun) {
            fclose(file);//ファイルを閉じる
            ev3_motor_stop(EV3_PORT_B, true);       // とめる
            ev3_motor_stop(EV3_PORT_C, true);
            isRun = false;
            break;
        } else {
            time = 0;               // initialize timer
            ev3_motor_set_power(EV3_PORT_B, 50);       // モーターをまわす
            ev3_motor_set_power(EV3_PORT_C, 50);
            isRun = true;
        }
 
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

void cyc1_task(intptr_t unused) {
    if (((++cyc1_count) % 2) == 0) {        // ledを点滅させる。タイマーが動いていることを確認する。
        ev3_led_set_color(LED_GREEN);
    } else {
        ev3_led_set_color(LED_OFF);
    }

    ev3_lcd_fill_rect(0, fonth * 1, EV3_LCD_WIDTH, fonth, EV3_LCD_WHITE);   // LCDにカラー値を表示
    char buf[100];
    int v = ev3_battery_voltage_mV();                           // batteryの電圧を読む
    sprintf(buf, "%ld:%ld, %d mV", time/60, time%60, v);        // mVでLCDに表示
    ev3_lcd_draw_string(buf, 0, fonth * 2);                     // 2行目に出力

    time++;     // increment timer 
}



/*
        メインタスク　- 駆動輪のモーターを制御して全身する。
    */
void main_task(intptr_t unused) {

    ev3_speaker_set_volume(50);//音量50に設定
    
    /* Register button handlers */
    ev3_button_set_on_clicked(ENTER_BUTTON, &button_clicked_handler, ENTER_BUTTON);

   /* Configure motors */
    ev3_motor_config(EV3_PORT_B, LARGE_MOTOR);   // メインタスクで制御：走行用 Lモーター
    ev3_motor_config(EV3_PORT_C, LARGE_MOTOR);   // 　　　　　　　　　　走行用 Lもーたー

    ev3_lcd_set_font(EV3_FONT_MEDIUM);
    ev3_font_get_size(EV3_FONT_MEDIUM, &fontw, &fonth);

    file=fopen(fname,"w");//ファイルをオープン(名前の指定)
    fprintf(file,"Battery check : time(sec), voltage 8mV)\n");//書き込み
    fclose(file);//ファイルを閉じる


    while(1) {
        int v = ev3_battery_voltage_mV();                           // batteryの電圧を読む
        file=fopen(fname,"a");//ファイルをオープン(名前の指定)
        fprintf(file,"%ld, %d\n", time, v);//書き込み
        fclose(file);

        dly_tsk(1000U * 1000U * 10U);
    }
}
