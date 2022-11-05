
/*
    Header file for wro 2022 robomission junior program
    wave.h
    Written by Naoto Watase and Kaita Suzuki.
    Copyright by NK Cells 2022. All rights reserved.
    This source code is a property of NK Cells. Nobody can use, copy, modify, transmit, redistibute and 
    sell without permission of NK Cells. 
*/

/*
	アプリケーションの定数の定義
    例：
#define MSEC (1000)
*/

/*
    肩の宣言
例：
typedef enum object {  
    PERSON = 10,
    CHILD = 3,
    ADULT = 2,
    FIRE = 5,
    CHEMICAL = 1,
    NOTHING = 0
} object_t ;
*/

// wave fileの指定
typedef enum wave {
    WAVE_0 = 0,
    WAVE_1,
    WAVE_2,
    WAVE_3,
    WAVE_4,
    WAVE_5,
    WAVE_6,
    WAVE_7,
    WAVE_8,
    WAVE_9,
    WAVE_10,
    WAVE_11,
    WAVE_12,
    WAVE_13,
    WAVE_14,
    WAVE_15,
    WAVE_16,
    WAVE_17,
    WAVE_Nashi,
    WAVE_Kuro,
    WAVE_Ao,
    WAVE_Midori,
    WAVE_Kiiro,
    WAVE_Aka,
    WAVE_Shiro,
    WAVE_Chairo,
} wave_t ;




/*
	アプリケーションのグローバル変数の宣言
    注意：
    変数の実体を定義するファイルで、このヘッダーファイルをインクルードする前に
    #define _MOTOR_H_
    と書いておく。(MOTORはそのファイル名)
*/
#ifndef _WAVE_H_ 
// グローバル変数をここでextern 宣言する。
// extern int test;

#endif

/*
 *  関数のプロトタイプ宣言
 */

extern void init_wave();
extern int play_wave(wave_t wave); 
